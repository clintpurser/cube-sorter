package cube_sorter

import (
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"

	"github.com/erh/vmodutils"
)

var Sorter = resource.NewModel("clint", "cube-sorter", "cube-sorter")

func init() {
	resource.RegisterService(generic.API, Sorter,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newSorter,
		},
	)
}

// sorter is a thin coordinator over one armWorker per configured arm. It owns
// the shared motion service and the motionMu that serializes arm motion so only
// one arm moves at a time.
type sorter struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()

	motion   motion.Service
	client   robot.Robot
	motionMu sync.Mutex

	workers []*armWorker
	wg      sync.WaitGroup
}

func newSorter(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewSorter(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSorter(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	robotClient, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		return nil, err
	}

	motionSvc, err := motion.FromProvider(deps, conf.motionService())
	if err != nil {
		return nil, err
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &sorter{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		motion:     motionSvc,
		client:     robotClient,
	}

	for _, unit := range conf.units() {
		w, err := s.buildWorker(deps, unit)
		if err != nil {
			cancelFunc()
			return nil, err
		}
		s.workers = append(s.workers, w)
	}

	for _, w := range s.workers {
		s.wg.Add(1)
		go w.run(&s.wg)
	}

	logger.Infof("cube-sorter started with %d arm(s)", len(s.workers))
	return s, nil
}

func (s *sorter) buildWorker(deps resource.Dependencies, unit ArmUnit) (*armWorker, error) {
	armDep, err := arm.FromProvider(deps, unit.Arm)
	if err != nil {
		return nil, err
	}
	gripperDep, err := gripper.FromProvider(deps, unit.Gripper)
	if err != nil {
		return nil, err
	}
	camDep, err := camera.FromProvider(deps, unit.Cam)
	if err != nil {
		return nil, err
	}
	segDep, err := vision.FromProvider(deps, unit.Segmenter)
	if err != nil {
		return nil, err
	}
	startPose, err := toggleswitch.FromProvider(deps, unit.StartPose)
	if err != nil {
		return nil, err
	}

	pitch := unit.BlockSize + unit.Margin
	zones := map[string]*zoneState{}
	for _, z := range unit.Zones {
		zones[z.Label] = &zoneState{
			cfg:   z,
			pitch: pitch,
		}
	}

	returnArea := &returnAreaState{
		cfg:   unit.ReturnArea,
		pitch: pitch,
	}

	return &armWorker{
		name:         unit.Arm,
		logger:       s.logger,
		arm:          armDep,
		cam:          camDep,
		gripper:      gripperDep,
		segmenter:    segDep,
		startPose:    startPose,
		zones:        zones,
		returnArea:   returnArea,
		cubeHeight:   unit.CubeHeight,
		graspZOffset: unit.GraspZOffset,
		approachYaw:  unit.ApproachYaw,
		motion:       s.motion,
		client:       s.client,
		motionMu:     &s.motionMu,
		parentCtx:    s.cancelCtx,
		cmdCh:        make(chan *cycleBarrier, 1),
		state:        stateIdle,
		phase:        phaseSorting,
	}, nil
}

func (s *sorter) Name() resource.Name {
	return s.name
}

func (s *sorter) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "start":
		s.logger.Infof("start sorting called on %d arm(s)", len(s.workers))
		barrier := newCycleBarrier(len(s.workers))
		for _, w := range s.workers {
			w.triggerWith(barrier)
		}
		return map[string]any{"success": true, "status": "started"}, nil

	case "stop":
		err := s.forEach(func(w *armWorker) error { return w.stop() })
		return map[string]any{"success": err == nil}, err

	case "reset":
		err := s.forEach(func(w *armWorker) error { return w.reset() })
		return map[string]any{"success": err == nil}, err

	case "get_detected_objects":
		result := map[string]any{}
		err := s.forEach(func(w *armWorker) error {
			objs, derr := w.detectOnly()
			result[w.name] = objs
			return derr
		})
		return map[string]any{"success": err == nil, "objects": result}, err

	case "pick_object":
		label, ok := cmd["label"].(string)
		if !ok {
			return nil, fmt.Errorf("pick_object requires a 'label' string parameter")
		}
		w := s.workerForLabel(label)
		if w == nil {
			return nil, fmt.Errorf("no configured arm owns label %q", label)
		}
		err := w.pickByLabel(label)
		return map[string]any{"success": err == nil}, err

	case "inspect_zone":
		label, ok := cmd["label"].(string)
		if !ok {
			return nil, fmt.Errorf("inspect_zone requires a 'label' string parameter")
		}
		w := s.workerForLabel(label)
		if w == nil {
			return nil, fmt.Errorf("no configured arm owns label %q", label)
		}
		err := w.inspectZoneByLabel(label)
		return map[string]any{"success": err == nil}, err

	case "get_status":
		return map[string]any{"success": true, "arms": s.aggregateStatus()}, nil
	}

	return nil, fmt.Errorf("unknown command: %v", cmd["command"])
}

// forEach runs fn on every worker and returns the first error (after running all).
func (s *sorter) forEach(fn func(*armWorker) error) error {
	var firstErr error
	for _, w := range s.workers {
		if err := fn(w); err != nil && firstErr == nil {
			firstErr = err
		}
	}
	return firstErr
}

func (s *sorter) workerForLabel(label string) *armWorker {
	for _, w := range s.workers {
		if w.ownsLabel(label) {
			return w
		}
	}
	return nil
}

func (s *sorter) aggregateStatus() map[string]any {
	out := map[string]any{}
	for _, w := range s.workers {
		out[w.name] = map[string]any{
			"status":           string(w.status()),
			"phase":            string(w.currentPhase()),
			"detected_objects": w.serializeDetected(),
		}
	}
	return out
}

func (s *sorter) Close(ctx context.Context) error {
	s.cancelFunc()
	s.wg.Wait()
	return s.client.Close(ctx)
}

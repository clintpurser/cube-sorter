package cube_sorter

import (
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// expectReady fails the test if ch isn't closed within the timeout.
func expectReady(t *testing.T, ch <-chan struct{}, timeout time.Duration, what string) {
	t.Helper()
	select {
	case <-ch:
	case <-time.After(timeout):
		t.Fatalf("%s: round never released within %v", what, timeout)
	}
}

// expectBlocked fails the test if ch is closed before the timeout.
func expectBlocked(t *testing.T, ch <-chan struct{}, timeout time.Duration, what string) {
	t.Helper()
	select {
	case <-ch:
		t.Fatalf("%s: round released unexpectedly", what)
	case <-time.After(timeout):
	}
}

// TestConvergeUnanimousEmpty: when every arm reports empty, the round releases
// with the "all empty" verdict that tells the arms the phase is finished.
func TestConvergeUnanimousEmpty(t *testing.T) {
	b := newConvergeBarrier(2)
	ra := b.report(true)
	expectBlocked(t, ra.ready, 20*time.Millisecond, "A-alone")
	rb := b.report(true)
	expectReady(t, ra.ready, 100*time.Millisecond, "A after B")
	expectReady(t, rb.ready, 100*time.Millisecond, "B")
	if !ra.allEmpty || !rb.allEmpty {
		t.Errorf("both empty should yield allEmpty=true, got A=%v B=%v", ra.allEmpty, rb.allEmpty)
	}
}

// TestConvergeNonUnanimousKeepsGoing: if any arm still picked a block this
// round, the verdict is not-all-empty, so the convergence loop runs again.
func TestConvergeNonUnanimousKeepsGoing(t *testing.T) {
	b := newConvergeBarrier(2)
	ra := b.report(true)  // A's source empty
	rb := b.report(false) // B still cleared a block
	expectReady(t, ra.ready, 100*time.Millisecond, "A")
	expectReady(t, rb.ready, 100*time.Millisecond, "B")
	if ra.allEmpty || rb.allEmpty {
		t.Errorf("one arm non-empty should yield allEmpty=false, got A=%v B=%v", ra.allEmpty, rb.allEmpty)
	}
}

// TestConvergeRearmsAcrossRounds is the core regression guard: the barrier is
// reused for every round, so each round must use a fresh channel and recompute
// its own verdict independently of the last.
func TestConvergeRearmsAcrossRounds(t *testing.T) {
	b := newConvergeBarrier(2)

	// Round 1: not unanimous (B picked a block).
	a1 := b.report(true)
	b1 := b.report(false)
	expectReady(t, a1.ready, 100*time.Millisecond, "round1 A")
	expectReady(t, b1.ready, 100*time.Millisecond, "round1 B")
	if a1.allEmpty {
		t.Error("round1 should not be all-empty")
	}

	// Round 2: must rearm with a fresh channel and block the early arm again.
	a2 := b.report(true)
	expectBlocked(t, a2.ready, 20*time.Millisecond, "round2 A-alone")
	b2 := b.report(true)
	expectReady(t, a2.ready, 100*time.Millisecond, "round2 A after B")
	if a2.ready == a1.ready {
		t.Error("round2 should use a fresh channel, got round1's")
	}
	if !a2.allEmpty || !b2.allEmpty {
		t.Error("round2 both empty should be all-empty")
	}
}

// TestConvergeFastArmCannotLap covers the reported failure mode: a faster arm
// must not advance to the next round before the partner has reported the
// current one. It can be at most one report ahead (queued, blocked on ready).
func TestConvergeFastArmCannotLap(t *testing.T) {
	b := newConvergeBarrier(2)

	rounds := 5
	aCompleted := make(chan int, rounds)
	go func() {
		for i := 0; i < rounds; i++ {
			r := b.report(false)
			<-r.ready
			aCompleted <- i
		}
	}()

	for i := 0; i < rounds; i++ {
		select {
		case completed := <-aCompleted:
			t.Fatalf("round %d: A completed before B reported (got %d)", i, completed)
		case <-time.After(20 * time.Millisecond):
		}
		b.report(false)
		select {
		case completed := <-aCompleted:
			if completed != i {
				t.Fatalf("round %d: A completed wrong round (%d)", i, completed)
			}
		case <-time.After(100 * time.Millisecond):
			t.Fatalf("round %d: A never observed B's report", i)
		}
	}
}

// TestConvergeLeaveReleasesWaitingPartner: an arm that bails (stop) while the
// partner is already parked at the round must release the partner, not hang.
func TestConvergeLeaveReleasesWaitingPartner(t *testing.T) {
	b := newConvergeBarrier(2)
	ra := b.report(true)
	expectBlocked(t, ra.ready, 20*time.Millisecond, "A-alone before leave")
	b.leave() // B bails out mid-round
	expectReady(t, ra.ready, 100*time.Millisecond, "A after B left")
}

// TestConvergeLeaveBeforeReportReleasesNext: the bailed arm leaves before its
// partner even reports; the partner must then proceed immediately, solo.
func TestConvergeLeaveBeforeReportReleasesNext(t *testing.T) {
	b := newConvergeBarrier(2)
	b.leave() // B bails before A reports
	ra := b.report(true)
	expectReady(t, ra.ready, 100*time.Millisecond, "A solo after B pre-left")
}

// TestConvergeSurvivorContinuesSolo: after a partner leaves, the survivor keeps
// reconciling solo, each round releasing immediately on its own report.
func TestConvergeSurvivorContinuesSolo(t *testing.T) {
	b := newConvergeBarrier(2)
	a1 := b.report(true)
	b1 := b.report(true)
	expectReady(t, a1.ready, 100*time.Millisecond, "round1 A")
	expectReady(t, b1.ready, 100*time.Millisecond, "round1 B")

	b.leave() // B departs after round 1

	for i := 0; i < 5; i++ {
		r := b.report(false)
		expectReady(t, r.ready, 100*time.Millisecond, "solo round")
	}
}

// TestConvergeBothLeaveInert: both arms leave; a stray extra leave is a no-op
// and a late report must not deadlock on an abandoned barrier.
func TestConvergeBothLeaveInert(t *testing.T) {
	b := newConvergeBarrier(2)
	b.leave()
	b.leave()
	b.leave() // idempotent past zero
	r := b.report(true)
	expectReady(t, r.ready, 100*time.Millisecond, "report on inert barrier")
}

// TestConvergeConcurrentRounds hammers the barrier with many concurrent reports
// across many rounds to catch races in the rearm logic.
func TestConvergeConcurrentRounds(t *testing.T) {
	const participants = 4
	const rounds = 50
	b := newConvergeBarrier(participants)

	var passed int64
	var wg sync.WaitGroup
	wg.Add(participants)
	for i := 0; i < participants; i++ {
		go func() {
			defer wg.Done()
			for c := 0; c < rounds; c++ {
				r := b.report(false)
				<-r.ready
				atomic.AddInt64(&passed, 1)
			}
		}()
	}

	done := make(chan struct{})
	go func() {
		wg.Wait()
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatalf("concurrent hammer deadlocked at %d/%d reports",
			atomic.LoadInt64(&passed), int64(participants*rounds))
	}
	if got, want := atomic.LoadInt64(&passed), int64(participants*rounds); got != want {
		t.Errorf("passed=%d, want %d", got, want)
	}
}

// TestConvergeLeaveDuringWaitRace covers the race where one arm is waiting on a
// round while the partner concurrently leaves: the waiter must be released.
func TestConvergeLeaveDuringWaitRace(t *testing.T) {
	for trial := 0; trial < 50; trial++ {
		b := newConvergeBarrier(2)
		r := b.report(true)
		go func() {
			b.leave()
		}()
		expectReady(t, r.ready, 200*time.Millisecond, "trial wait")
	}
}

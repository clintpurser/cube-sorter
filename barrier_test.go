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
		t.Fatalf("%s: barrier never released within %v", what, timeout)
	}
}

// expectBlocked fails the test if ch is closed before the timeout.
func expectBlocked(t *testing.T, ch <-chan struct{}, timeout time.Duration, what string) {
	t.Helper()
	select {
	case <-ch:
		t.Fatalf("%s: barrier released unexpectedly", what)
	case <-time.After(timeout):
	}
}

// TestBarrierTwoCyclesRendezvous is the core regression test for the bug
// where the second cycle stopped synchronizing across arms. Both arms must
// rendezvous on every cycle when they reuse the same barrier — not just the
// first.
func TestBarrierTwoCyclesRendezvous(t *testing.T) {
	b := newCycleBarrier(2)

	// Cycle 1: A arrives first, must wait for B.
	readyA1 := b.arrive()
	expectBlocked(t, readyA1, 20*time.Millisecond, "cycle1 A-alone")
	readyB1 := b.arrive()
	expectReady(t, readyA1, 100*time.Millisecond, "cycle1 A after B arrived")
	expectReady(t, readyB1, 100*time.Millisecond, "cycle1 B")

	// Cycle 2: must rearm and block A again until B arrives. This is the
	// behavior the original one-shot barrier did NOT provide on auto-loop.
	readyA2 := b.arrive()
	expectBlocked(t, readyA2, 20*time.Millisecond, "cycle2 A-alone")
	readyB2 := b.arrive()
	expectReady(t, readyA2, 100*time.Millisecond, "cycle2 A after B arrived")
	expectReady(t, readyB2, 100*time.Millisecond, "cycle2 B")

	if readyA2 == readyA1 {
		t.Error("cycle2 should use a fresh ready channel, got the cycle1 one")
	}
}

// TestBarrierFastArmCannotLap covers the exact failure mode the user
// reported: a faster arm trying to enter return before the partner has
// finished sorting. The faster arm must block at every cycle's barrier.
func TestBarrierFastArmCannotLap(t *testing.T) {
	b := newCycleBarrier(2)

	// Simulate arm A racing through cycles. After each rendezvous it
	// immediately arrives for the next one.
	cycles := 5
	aCompleted := make(chan int, cycles)
	go func() {
		for i := 0; i < cycles; i++ {
			<-b.arrive()
			aCompleted <- i
		}
	}()

	// B is slow: only arrives after observing A has completed cycle i.
	for i := 0; i < cycles; i++ {
		// A must NOT be able to complete cycle i without B arriving.
		select {
		case completed := <-aCompleted:
			t.Fatalf("cycle %d: A completed before B arrived (got %d)", i, completed)
		case <-time.After(20 * time.Millisecond):
		}
		<-b.arrive()
		select {
		case completed := <-aCompleted:
			if completed != i {
				t.Fatalf("cycle %d: A completed wrong cycle (%d)", i, completed)
			}
		case <-time.After(100 * time.Millisecond):
			t.Fatalf("cycle %d: A never observed B's arrival", i)
		}
	}
}

// TestBarrierLeaveReleasesWaitingPartner covers the case where one arm bails
// out (sort empty / pick failed past restart / interrupt) while the partner
// is already parked at the barrier. The partner must not hang.
func TestBarrierLeaveReleasesWaitingPartner(t *testing.T) {
	b := newCycleBarrier(2)
	readyA := b.arrive()
	expectBlocked(t, readyA, 20*time.Millisecond, "A-alone before leave")
	b.leave() // B bails out mid-cycle
	expectReady(t, readyA, 100*time.Millisecond, "A after B left")
}

// TestBarrierLeaveBeforeArrivalReleasesNext covers the case where the bailed
// arm leaves before its partner even arrives at the rendezvous. The partner,
// once it does arrive, must proceed immediately as a solo participant.
func TestBarrierLeaveBeforeArrivalReleasesNext(t *testing.T) {
	b := newCycleBarrier(2)
	b.leave() // B bails before A arrives
	readyA := b.arrive()
	expectReady(t, readyA, 100*time.Millisecond, "A solo after B pre-left")
}

// TestBarrierSurvivorContinuesAcrossCycles covers the post-bail steady state:
// after the partner leaves, the surviving arm should continue cycling solo
// without ever blocking at the rendezvous.
func TestBarrierSurvivorContinuesAcrossCycles(t *testing.T) {
	b := newCycleBarrier(2)
	// Cycle 1: both arrive normally.
	readyA1 := b.arrive()
	readyB1 := b.arrive()
	expectReady(t, readyA1, 100*time.Millisecond, "cycle1 A")
	expectReady(t, readyB1, 100*time.Millisecond, "cycle1 B")

	// B bails after cycle 1's return.
	b.leave()

	// Cycle 2..N: A continues solo, each cycle must release immediately.
	for i := 0; i < 5; i++ {
		ready := b.arrive()
		expectReady(t, ready, 100*time.Millisecond, "solo cycle")
	}
}

// TestBarrierBothLeaveLeavesBarrierInert verifies the abandoned-barrier
// state: both arms leave, nobody is waiting, and a hypothetical late arrival
// doesn't block on a barrier that has no remaining expected participants.
func TestBarrierBothLeaveLeavesBarrierInert(t *testing.T) {
	b := newCycleBarrier(2)
	b.leave()
	b.leave()
	// Either an extra leave or an arrive on an inert barrier must not deadlock.
	b.leave() // idempotent past zero
	ready := b.arrive()
	expectReady(t, ready, 100*time.Millisecond, "arrive on inert barrier")
}

// TestBarrierConcurrentCycles hammers the barrier with many concurrent
// arrivals across many cycles to catch races in the rearm logic.
func TestBarrierConcurrentCycles(t *testing.T) {
	const participants = 4
	const cycles = 50
	b := newCycleBarrier(participants)

	var passed int64
	var wg sync.WaitGroup
	wg.Add(participants)
	for i := 0; i < participants; i++ {
		go func() {
			defer wg.Done()
			for c := 0; c < cycles; c++ {
				ready := b.arrive()
				<-ready
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
		t.Fatalf("concurrent barrier hammer deadlocked at %d/%d arrivals",
			atomic.LoadInt64(&passed), int64(participants*cycles))
	}
	if got, want := atomic.LoadInt64(&passed), int64(participants*cycles); got != want {
		t.Errorf("passed=%d, want %d", got, want)
	}
}

// TestBarrierLeaveDuringWaitWithPartnerArriving covers the race where one arm
// is waiting on the ready channel while the partner concurrently leaves: the
// waiter must observe the release, not hang forever.
func TestBarrierLeaveDuringWaitWithPartnerArriving(t *testing.T) {
	for trial := 0; trial < 50; trial++ {
		b := newCycleBarrier(2)
		ready := b.arrive()
		// Concurrently leave; the waiter should be released either way.
		go func() {
			b.leave()
		}()
		expectReady(t, ready, 200*time.Millisecond, "trial wait")
	}
}

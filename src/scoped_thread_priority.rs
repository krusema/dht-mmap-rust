/// When a `HighThreadPriorityGuard` is created, thread scheduling is set to the highest `SCHED_FIFO`
/// priority using libc. When the struct is dropped, the priority is set to 0 (default) and the scheduling policy
/// that was present before the drop.
pub struct HighThreadPriorityGuard {
    policy_before: libc::c_int,
}

impl HighThreadPriorityGuard {
    pub fn new() -> Self {
        unsafe {
            let before = libc::sched_getscheduler(0);

            let scheduler = libc::sched_param {
                sched_priority: libc::sched_get_priority_max(libc::SCHED_FIFO),
            };

            libc::sched_setscheduler(0, libc::SCHED_FIFO, &scheduler);

            return Self {
                policy_before: before,
            };
        }
    }
}

impl Drop for HighThreadPriorityGuard {
    fn drop(&mut self) {
        let scheduler = libc::sched_param { sched_priority: 0 };
        unsafe {
            libc::sched_setscheduler(0, self.policy_before, &scheduler);
        }
    }
}

"""Linux parent-death guard; construct only before starting threaded runtimes."""
import ctypes
import os
import signal

# Resolve libc before fork. preexec_fn must be used only before ROS starts threads.
_libc = ctypes.CDLL(None, use_errno=True)
_prctl = _libc.prctl
_prctl.argtypes = (ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong)
_prctl.restype = ctypes.c_int
_PR_SET_PDEATHSIG = 1


def parent_death_preexec():
    parent_pid = os.getpid()

    def guard():
        if _prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0) != 0 or os.getppid() != parent_pid:
            os._exit(127)

    return guard

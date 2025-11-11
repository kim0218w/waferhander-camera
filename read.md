 File "/usr/lib/python3/dist-packages/matplotlib/backend_bases.py", line 3547, in show
    cls.mainloop()
    ~~~~~~~~~~~~^^
  File "/usr/lib/python3/dist-packages/matplotlib/backends/backend_qt.py", line 642, in start_main_loop
    with _allow_interrupt_qt(qapp):
         ~~~~~~~~~~~~~~~~~~~^^^^^^
  File "/usr/lib/python3.13/contextlib.py", line 141, in __enter__
    return next(self.gen)
  File "/usr/lib/python3/dist-packages/matplotlib/backend_bases.py", line 1641, in _allow_interrupt
    old_wakeup_fd = signal.set_wakeup_fd(wsock.fileno())
ValueError: set_wakeup_fd only works in main thread of the main interpreter
[INFO] 저장할 측정 데이터가 없습니다.

content = open('auto_tune.py').read()

# Fix kill_all indentation
content = content.replace("global _rc_override_stop, _rc_thread\n    if _rc_thread", "    global _rc_override_stop, _rc_thread\n    if _rc_thread")

# Fix start_stack indentation
content = content.replace('print(f"  [stack] Starting RC override thread...")', '    print(f"  [stack] Starting RC override thread...")')

open('auto_tune.py', 'w').write(content)
print("Indentation fixed.")

extends SceneTree
# Not a test: with a DualSense connected, sets R2's strongest click and prints how it went;
# press R2 within five seconds to feel it (the triggers are freed on exit).
# Run: godot --headless --path . --script res://tests/probe_trigger.gd
func _initialize():
	call_deferred("run")
func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	print("trigger probe: ", lab.ProbeTrigger())
	await create_timer(float(OS.get_environment("HOLD")) if OS.get_environment("HOLD") != "" else 5.0).timeout
	quit()

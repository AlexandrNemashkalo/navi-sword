extends SceneTree
# Not a test: screenshots of a walk, seen from the side. Run with a window:
# SHOT_DIR=<folder> godot --path . --resolution 1280x720 --script res://tests/shot_walk.gd
func _initialize():
	call_deferred("run")
func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	var out = OS.get_environment("SHOT_DIR")
	lab.Orbit = float(OS.get_environment("ORBIT")) if OS.get_environment("ORBIT") != "" else 90.0
	for i in 30:
		await process_frame
	lab.WalkInput = Vector2(float(OS.get_environment("SX")) if OS.get_environment("SX") != "" else 0.0, -1)
	for k in 6:
		for i in 7:
			await process_frame
		root.get_viewport().get_texture().get_image().save_png(out + "/walk_%d.png" % k)
	quit()

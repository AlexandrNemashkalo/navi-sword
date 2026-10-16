extends SceneTree
# Not a test: screenshots of the default view (a guard, a raised cut, one more, then opaque).
# Run with a window: SHOT_DIR=<folder> godot --path . --resolution 1280x720 --script res://tests/shot_camera.gd
func _initialize():
	call_deferred("run")
func frames(n):
	for i in n:
		await process_frame
func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	var out = OS.get_environment("SHOT_DIR")
	lab.CentrePad()
	for k in 3:
		var pad = [Vector3(0, 0, 0), Vector3(30, 25, 0), Vector3(10, -15, 0)][k]
		for i in 60:
			lab.DrivePad(pad, Vector2.ZERO, 0.0, false)
			await process_frame
		root.get_viewport().get_texture().get_image().save_png(out + "/view_%d.png" % k)
	lab.SeeThrough = false
	lab.call("ApplySeeThrough")
	await frames(5)
	root.get_viewport().get_texture().get_image().save_png(out + "/view_opaque.png")
	quit()

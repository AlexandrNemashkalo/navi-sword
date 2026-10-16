extends SceneTree
# Not a test: screenshots of fine work's guard and the thrust's lunge, from the side. Run with a window:
# SHOT_DIR=<folder> godot --path . --resolution 1280x720 --script res://tests/shot_thrust.gd
func _initialize():
	call_deferred("run")
func step(lab, r2, fine):
	lab.DrivePad(Vector3.ZERO, Vector2.ZERO, r2, fine)
	await process_frame
func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var out = OS.get_environment("SHOT_DIR")
	lab.Orbit = float(OS.get_environment("ORBIT")) if OS.get_environment("ORBIT") != "" else 100.0
	lab.CentrePad()
	for i in 90:
		lab.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
		await process_frame
	for i in 45:
		lab.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, true)
		lab._Process(1.0 / 60)
		await process_frame
	root.get_viewport().get_texture().get_image().save_png(out + "/thrust_guard.png")
	for i in 40:
		lab.DrivePad(Vector3.ZERO, Vector2.ZERO, min(1.0, i / 10.0), true)
		lab._Process(1.0 / 60)
		await process_frame
	root.get_viewport().get_texture().get_image().save_png(out + "/thrust_lunge.png")
	quit()

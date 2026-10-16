extends SceneTree

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	lab.WalkInput = Vector2.ZERO
	for i in 90:
		lab.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	var skeleton = lab.find_children("*", "Skeleton3D", true, false)[0]
	var head = skeleton.find_bone("head")
	var head_start = (skeleton.global_transform * skeleton.get_bone_global_pose(head)).origin
	var feet = [lab.FootPosition(0), lab.FootPosition(1)]
	var arm = []
	for d in 7:
		arm.append(lab.Target(0, d))
	lab.DrivePadDodging(Vector3.ZERO, Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
	var arm_error = 0.0
	for i in 120:
		lab.DrivePadDodging(Vector3(-22, 15, 20), Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
		lab._Process(1.0 / 60)
		for d in 7:
			arm_error = max(arm_error, abs(lab.Target(0, d) - arm[d]))
	var head_end = (skeleton.global_transform * skeleton.get_bone_global_pose(head)).origin
	var step = max(feet[0].distance_to(lab.FootPosition(0)), feet[1].distance_to(lab.FootPosition(1)))
	var foot_error = 0.0
	for i in 2:
		var bone = skeleton.find_bone("foot.l" if i == 0 else "foot.r")
		var actual = (skeleton.global_transform * skeleton.get_bone_global_pose(bone)).origin
		foot_error = max(foot_error, actual.distance_to(lab.FootPosition(i)))
	print("Dodge angles: %s; head moved %.3f m; foot step %.3f; IK error %.4f; arm targets %.4f" % [lab.DodgeAngles, head_start.distance_to(head_end), step, foot_error, arm_error])
	var ok = lab.DodgeAngles.x > 20 and lab.DodgeAngles.z > 20 and head_start.distance_to(head_end) > 0.1 and step > 0.04 and foot_error < 0.035 and arm_error < 0.01
	if OS.get_environment("SHOT_DIR") != "":
		await process_frame
		await process_frame
		root.get_viewport().get_texture().get_image().save_png(OS.get_environment("SHOT_DIR") + "/dodge.png")
	for i in 120:
		lab.DrivePadDodging(Vector3(-22, 15, 20), Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
		lab._Process(1.0 / 60)
		for d in 7:
			arm_error = max(arm_error, abs(lab.Target(0, d) - arm[d]))
	ok = ok and lab.DodgeAngles.length() < 0.01 and lab.DodgeOffset.length() < 0.001 and arm_error < 0.01
	# Keep L2 down throughout: both a one-frame R1 tap and a long regrip must
	# rebase to neutral, never to the residual tilt of the unfinished return.
	var centre_error = 0.0
	var response_error = 0.0
	for hold_frames in [1, 30, 1]:
		lab.DrivePadDodging(Vector3.ZERO, Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
		for i in 90:
			lab.DrivePadDodging(Vector3(-20, 15, 18), Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
			lab._Process(1.0 / 60)
		var grip = Vector3(35, -25, -30)
		for i in hold_frames:
			lab.DrivePadDodging(grip * (float(i + 1) / hold_frames), Vector2.ZERO, 0.0, false, true, 1.0, 1.0 / 60)
			lab._Process(1.0 / 60)
		for i in 90:
			lab.DrivePadDodging(grip, Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
			lab._Process(1.0 / 60)
		centre_error = max(centre_error, lab.DodgeAngles.length())
		centre_error = max(centre_error, lab.DodgeOffset.length())
		for i in 90:
			lab.DrivePadDodging(grip + Vector3(-10, 5, 10), Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
			lab._Process(1.0 / 60)
		response_error = max(response_error, lab.DodgeAngles.distance_to(Vector3(12, 5, 12)))
		lab.DrivePadDodging(grip, Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
	print("L2 + R1: neutral error %.4f; new-centre response error %.4f" % [centre_error, response_error])
	ok = ok and centre_error < 0.01 and response_error < 0.01
	# Direct sliders use the same limits and leg solver, without a connected pad.
	lab.SetDodge(100, -100, 100, -1, 1)
	for i in 150:
		lab._Process(1.0 / 60)
	ok = ok and lab.DodgeAngles.is_equal_approx(Vector3(35, 30, -25)) and not lab.FeetMoving
	if ok:
		print("PASS dodge: gyro routing, planted-foot IK, corrective steps, release and manual limits")
		quit()
	else:
		push_error("Dodge regression")
		quit(1)

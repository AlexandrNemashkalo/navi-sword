extends SceneTree
# R1 held, the right stick turns the camera round the fighter; the arm does not move. Let go
# within a tap's time after turning the camera, the sword does not go back to guard. Let go with
# the stick still over, the arm does not move until the stick has been back at the centre; then
# the stick works again. R3 puts the camera back behind.
# Run: godot --headless --path . --script res://tests/joint_camera_stick_test.gd

func _initialize():
	call_deferred("run")

func frame(lab, pad, stick, r1, r3 = false):
	var s = lab.CameraStickFor(stick, r1, r3, 1.0 / 60)
	lab.DrivePadClutched(pad, s, 0.0, false, r1)

func targets(lab):
	var t = []
	for d in 7:
		t.append(lab.Target(0, d))
	return t

func moved(a, b):
	var most = 0.0
	for d in 7:
		most = max(most, abs(a[d] - b[d]))
	return most

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	var pad = Vector3(20, -10, 0)
	for i in 60:
		frame(lab, pad, Vector2.ZERO, false)
	var held = targets(lab)
	var orbit = lab.Orbit
	# A tap's length (0.17 s) of R1 with the stick right.
	for i in 10:
		frame(lab, pad, Vector2(1, 0), true)
	var turned = wrapf(lab.Orbit - orbit, -180.0, 180.0)
	var during = moved(held, targets(lab))
	# Let go, the stick still right, then a while.
	for i in 40:
		frame(lab, pad, Vector2(1, 0), false)
	var after = moved(held, targets(lab))
	# The stick back to the centre, then down: the elbow bends again.
	for i in 5:
		frame(lab, pad, Vector2.ZERO, false)
	for i in 10:
		frame(lab, pad, Vector2(0, 1), false)
	var elbow = lab.Target(0, 3) - held[3]
	frame(lab, pad, Vector2.ZERO, false, true)
	var back = lab.Orbit
	print("camera turned %.0f°; arm moved under R1 %.2f°, after letting go %.2f°; elbow then %.0f°; R3: camera at %.0f°" % [turned, during, after, elbow, back])
	if turned > 20 and during < 0.01 and after < 0.01 and elbow > 20 and abs(back - orbit) < 0.01:
		print("PASS camera stick: R1 + right stick turns the camera, the arm stays, R3 puts it back")
		quit()
	else:
		push_error("R1 + right stick does not turn the camera cleanly")
		quit(1)

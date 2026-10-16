extends SceneTree
# Dodging (L2 held), R1 and the right stick turn the camera: the torso keeps its dodge and the
# arm stays; let go of R1, the dodge goes on from where it was, without a jump. R1 under L2
# without the stick is still the regrip: the torso comes back to neutral.
# Run: godot --headless --path . --script res://tests/joint_dodge_camera_test.gd

func _initialize():
	call_deferred("run")

func frame(lab, pad, stick, r1, l2):
	var s = lab.CameraStickFor(stick, r1, false, 1.0 / 60)
	lab.DrivePadDodging(pad, s, 0.0, false, r1, l2, 1.0 / 60)
	lab._Process(1.0 / 60)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	for i in 30:
		frame(lab, Vector3.ZERO, Vector2.ZERO, false, 0.0)
	frame(lab, Vector3.ZERO, Vector2.ZERO, false, 1.0)
	var pad = Vector3(-20, 10, 15)
	for i in 90:
		frame(lab, pad, Vector2.ZERO, false, 1.0)
	var dodged = lab.DodgeAngles
	var arm = lab.Target(0, 0)
	var orbit = lab.Orbit
	for i in 30:
		frame(lab, pad + Vector3(0, i * 0.5, 0), Vector2(1, 0), true, 1.0)
	var turned = wrapf(lab.Orbit - orbit, -180.0, 180.0)
	var kept = lab.DodgeAngles.distance_to(dodged)
	var arm_kept = abs(lab.Target(0, 0) - arm)
	# Let go of R1 (stick back), L2 still held: the dodge goes on from the pad as it is now.
	var moved = pad + Vector3(0, 15, 0)
	for i in 30:
		frame(lab, moved, Vector2.ZERO, false, 1.0)
	var after = lab.DodgeAngles.distance_to(dodged)
	# R1 under L2 without the stick, held: the regrip straightens the torso.
	for i in 40:
		frame(lab, moved, Vector2.ZERO, true, 1.0)
	for i in 60:
		frame(lab, moved, Vector2.ZERO, false, 1.0)
	var neutral = lab.DodgeAngles.length()
	print("camera turned %.0f°; dodge moved under R1 %.3f°, after %.3f°; arm %.3f°; regrip leaves %.3f°" % [turned, kept, after, arm_kept, neutral])
	if turned > 60 and kept < 0.05 and after < 0.05 and arm_kept < 0.01 and neutral < 0.05:
		print("PASS dodge camera: L2 + R1 + right stick turns the camera, the dodge stays")
		quit()
	else:
		push_error("Turning the camera while dodging loses the dodge")
		quit(1)

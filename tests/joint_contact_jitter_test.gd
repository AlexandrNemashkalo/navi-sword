extends SceneTree
# Plays the gyro where the sword arm runs into the body — across the chest with the elbow bent,
# the forearm rolled into the belly, wound up behind the back — with a little sensor noise, and
# counts how often the blade's tip and the hand reverse their motion. Pressed against the body
# the arm used to see-saw: the solver pushed it out, the next step pressed it back in.
# Run: godot --headless --path . --script res://tests/joint_contact_jitter_test.gd

func _initialize():
	call_deferred("run")

func tip(lab):
	var s = lab.get("Sword").global_transform
	return s.origin + s.basis.y * 0.9

func hand(lab):
	var sk = lab.find_children("*", "Skeleton3D", true, false)[0]
	return sk.global_transform * sk.get_bone_global_pose(sk.find_bone("hand.r")).origin

func trial(lab, label, pad_of, stick):
	var rng = RandomNumberGenerator.new()
	rng.seed = 4
	lab.CentrePad()
	for i in 60:
		lab.DrivePad(pad_of.call(0), stick, 0.0, false)
		lab._Process(1.0 / 60)
	var pt = tip(lab)
	var ph = hand(lab)
	var vt = Vector3.ZERO
	var vh = Vector3.ZERO
	var tip_flips = 0
	var hand_flips = 0
	for f in 240:
		var p = pad_of.call(f) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3))
		lab.DrivePad(p, stick, 0.0, false)
		lab._Process(1.0 / 60)
		var nt = tip(lab)
		var nh = hand(lab)
		var v = nt - pt
		var w = nh - ph
		# A visible reversal: the tip moving a centimetre a frame one way, then the other.
		if v.length() > 0.01 and vt.length() > 0.01 and v.dot(vt) < 0:
			tip_flips += 1
		if w.length() > 0.002 and vh.length() > 0.002 and w.dot(vh) < 0:
			hand_flips += 1
		if v.length() > 0.01:
			vt = v
		if w.length() > 0.002:
			vh = w
		pt = nt
		ph = nh
	print("%s: tip reversals %d, hand reversals %d" % [label, tip_flips, hand_flips])
	return tip_flips <= 6 and hand_flips <= 6

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	# Each pad motion below reverses once or twice in four seconds by itself.
	var ok = trial(lab, "across the chest, elbow bent", func(f): return Vector3(0, -25.0 * sin(f * TAU / 240), 0), Vector2(0, 1))
	ok = trial(lab, "forearm into the belly      ", func(f): return Vector3(-10, -10, -60.0 * abs(sin(f * TAU / 240))), Vector2(0, 0.8)) and ok
	ok = trial(lab, "behind the back, elbow bent ", func(f): return Vector3(10, 30.0 + 30.0 * sin(f * TAU / 240), 0), Vector2(0, 1)) and ok
	if ok:
		print("PASS arm against the body: no see-sawing")
		quit()
	else:
		push_error("The arm see-saws against the body")
		quit(1)

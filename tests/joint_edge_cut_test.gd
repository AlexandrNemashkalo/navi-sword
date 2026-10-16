extends SceneTree
# A cut from the right: the pad wound up to the right, then swung left fast with R2. The forearm
# turns so an edge leads the way the tip goes while the pad swings (against the same cut with the
# help off; counted from 6 m/s, where a cut whistles), and the arm comes on across the chest: past
# 150° from the front, where it stopped at 137° (the arm met the chest at 78° across it before the
# shoulder came forward).
# Run: godot --headless --path . --script res://tests/joint_edge_cut_test.gd

func _initialize():
	call_deferred("run")

func step(lab, pad, r2):
	lab.DrivePad(pad, Vector2.ZERO, r2, false)
	lab._Process(1.0 / 60)

func tip(lab):
	var t = lab.Sword.global_transform
	return t.origin + t.basis.y * 1.31

func cut(lab):
	lab.CentrePad()
	for i in 90:
		step(lab, Vector3.ZERO, 0.0)
	# Wound up: turned right and nose a little up, the elbow bent.
	for f in 40:
		step(lab, Vector3(8, 25.0 * min(1.0, f / 20.0), 0), 0.0)
	var before = tip(lab)
	var sum = 0.0
	var frames = 0
	var most = -999.0
	var help = 0.0
	for f in 30:
		var k = min(1.0, f / 18.0)
		step(lab, Vector3(8.0 - 10.0 * k, 25.0 - 55.0 * k, 0), min(1.0, f / 6.0))
		var now = tip(lab)
		var v = (now - before) * 60.0
		before = now
		most = max(most, lab.PlaneFromFront(0))
		help = max(help, abs(lab.EdgeHelp))
		var t = lab.Sword.global_transform
		var along = t.basis.y.normalized()
		var across = v - along * v.dot(along)
		if f < 20 and v.length() > 6.0:
			sum += abs(t.basis.x.normalized().dot(across)) / v.length()
			frames += 1
	for i in 30:
		step(lab, Vector3(-2, -30, 0), 1.0)
		most = max(most, lab.PlaneFromFront(0))
	return [sum / max(frames, 1), frames, most, help]

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.EdgeAssist = false
	var off = cut(lab)
	lab.EdgeAssist = true
	var on = cut(lab)
	print("edge leading: help off %.2f, on %.2f (%d fast frames, forearm turned up to %.0f°); plane across %.0f°" % [off[0], on[0], on[1], on[3], on[2]])
	if on[1] >= 5 and on[0] > 0.75 and on[0] > off[0] + 0.1 and on[2] > 150:
		print("PASS edge cut: the cut from the right lands on the edge, the arm well across")
		quit()
	else:
		push_error("The cut from the right is flat or short")
		quit(1)

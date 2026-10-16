extends SceneTree
# The end of a cut from the right: the pad swung left fast and stopped. With the follow-through
# the wrist bends on as the shoulder stops, so the tip runs on the way it was going further than
# with the wrist held; then it comes back to where it was.
# Run: godot --headless --path . --script res://tests/joint_follow_through_test.gd

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
	for f in 40:
		step(lab, Vector3(8, 25.0 * min(1.0, f / 20.0), 0), 0.0)
	var before = tip(lab)
	var way = Vector3.ZERO
	var fastest = 0.0
	var stop = Vector3.ZERO
	for f in 18:
		var k = min(1.0, f / 18.0)
		step(lab, Vector3(8.0 - 10.0 * k, 25.0 - 55.0 * k, 0), min(1.0, f / 6.0))
		var now = tip(lab)
		var v = (now - before) * 60.0
		before = now
		if v.length() > fastest:
			fastest = v.length()
			way = v.normalized()
	# The pad held where the cut ended: how far on does the tip go?
	var start = tip(lab)
	var on = 0.0
	var bend = 0.0
	for i in 30:
		step(lab, Vector3(-2, -30, 0), 1.0)
		on = max(on, (tip(lab) - start).dot(way))
		bend = max(bend, lab.FollowBend.length())
	for i in 180:
		step(lab, Vector3(-2, -30, 0), 1.0)
	return [on, bend, lab.FollowBend.length(), fastest]

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.FollowThrough = false
	var off = cut(lab)
	lab.FollowThrough = true
	var on = cut(lab)
	print("tip on after the swing: wrist held %.2f m, follow-through %.2f m (wrist bent up to %.0f°, back to %.1f° after 3 s; tip at %.1f m/s)" % [off[0], on[0], on[1], on[2], on[3]])
	if on[0] > off[0] + 0.1 and on[1] > 10 and on[2] < 0.5:
		print("PASS follow-through: the wrist carries the cut on and comes back")
		quit()
	else:
		push_error("The wrist does not carry the cut on, or does not come back")
		quit(1)

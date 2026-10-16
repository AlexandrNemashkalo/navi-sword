extends SceneTree
# The thrust: with L1 held, R2 drives the sword forward along its own line — the hand ahead
# along the blade, hardly off it, the blade keeping its direction — and back as R2 is let go.
# Without L1, R2 is the cut's end as before: the elbow straightens.
# Run: godot --headless --path . --script res://tests/joint_thrust_test.gd

func _initialize():
	call_deferred("run")

func step(lab, r2, fine):
	lab.DrivePad(Vector3.ZERO, Vector2.ZERO, r2, fine)
	lab._Process(1.0 / 60)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	for i in 120:
		step(lab, 0.0, false)
	for i in 30:
		step(lab, 0.0, true)
	var sword = lab.Sword
	var start = sword.global_transform
	var line = start.basis.y.normalized()
	var elbow_before = lab.Angle(0, 3)
	for f in 60:
		step(lab, min(1.0, f / 15.0), true)
	var asked = lab.ThrustReached
	var now = sword.global_transform
	var moved = now.origin - start.origin
	var along = moved.dot(line)
	var off = (moved - line * along).length()
	var turned = rad_to_deg(line.angle_to(now.basis.y.normalized()))
	var elbow_thrust = lab.Angle(0, 3)
	for f in 60:
		step(lab, 0.0, true)
	var back = sword.global_transform.origin.distance_to(start.origin)
	# Without L1: the elbow straightens.
	for i in 30:
		step(lab, 0.0, false)
	for f in 60:
		step(lab, min(1.0, f / 15.0), false)
	var elbow_cut = lab.Angle(0, 3)
	print("thrust: sword %.2f m along its line, %.2f m off it, turned %.1f°, elbow %.0f° -> %.0f° (asked %.2f along, %.2f off); let go: %.2f m from start; cut: elbow %.0f°" % [
		along, off, turned, elbow_before, elbow_thrust, asked.x, asked.y, back, elbow_cut])
	if along > 0.2 and off < 0.04 and turned < 5 and elbow_thrust < elbow_before - 15 and back < 0.03 and elbow_cut < 5:
		print("PASS thrust: L1 + R2 drives the sword along its line; R2 alone straightens the elbow")
		quit()
	else:
		push_error("The thrust does not drive the sword along its line")
		quit(1)

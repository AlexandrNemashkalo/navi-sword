extends SceneTree
# Not a test: sends the sound to the DualSense's speaker (USB), turns the speaker on, and plays
# a 440 Hz tone for three seconds. Run with a window: godot --path . --script res://tests/probe_audio.gd
func _initialize():
	call_deferred("run")
func run():
	print("audio devices: ", AudioServer.get_output_device_list())
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	print("speaker probe: ", lab.ProbeSpeaker())
	var player = AudioStreamPlayer.new()
	var generator = AudioStreamGenerator.new()
	generator.mix_rate = 44100
	player.stream = generator
	root.add_child(player)
	player.play()
	var playback = player.get_stream_playback()
	var phase = 0.0
	var until = Time.get_ticks_msec() + 3000
	while Time.get_ticks_msec() < until:
		for i in playback.get_frames_available():
			phase += TAU * 440.0 / 44100.0
			playback.push_frame(Vector2.ONE * 0.4 * sin(phase))
		await process_frame
	quit()

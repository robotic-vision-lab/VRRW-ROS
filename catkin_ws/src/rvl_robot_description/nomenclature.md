# Component name remapping

Per [instruction manual](/official_docs/Robotiq_2F_Documentation.pdf) of Robotiq 2F Gripper, joint/link names are renamed as follow:

`base_link` is renamed to `palm`

`outer_knuckle` is renamed to `knuckle`

`outer_finger` is renamed to `bar`

`inner_knuckle` is renamed to `proximal_phalanx`

`inner_finger` is renamed to `distal_phalanx`

`finger_pad` is renamed to `finger_tip`

All link/joint names will be prepended with `robotiq_2f` instead of the previous `robotiq_arg2f`.

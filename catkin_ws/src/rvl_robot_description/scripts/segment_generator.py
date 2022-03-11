# ---------------------------------------------------------------------------- #
#                                     LINKS                                    #
# ---------------------------------------------------------------------------- #

link_names = ['knuckle', 'bar', 'proximal_phalanx', 'distal_phalanx']

with open('link.xacro', 'w') as f:
    for name in link_names:
        f.write(f'<xacro:macro name="{name}_link" params="prefix fingerprefix">\n')
        f.write(f'    <link name="${{prefix}}${{fingerprefix}}${{{name}_link_name}}">\n')
        f.write(f'        <visual>\n')
        f.write(f'            <origin xyz="${{{name}_visual_xyz}}" rpy="${{{name}_visual_rpy}}" />\n')
        f.write(f'            <geometry>\n')
        f.write(f'                <mesh filename="${{{name}_visual_mesh_file}}" scale="${{{name}_visual_mesh_scale}}" />\n')
        f.write(f'            </geometry>\n')
        f.write(f'            <material name="${{{name}_material_name}}">\n')
        f.write(f'                <color rgba="${{{name}_material_color}}" />\n')
        f.write(f'            </material>\n')
        f.write(f'        </visual>\n')
        f.write(f'        <collision>\n')
        f.write(f'            <origin xyz="${{{name}_collision_xyz}}" rpy="${{{name}_collision_rpy}}" />\n')
        f.write(f'            <geometry>\n')
        f.write(f'                <mesh filename="${{{name}_collision_mesh_file}}" scale="${{{name}_collision_mesh_scale}}" />\n')
        f.write(f'            </geometry>\n')
        f.write(f'        </collision>\n')
        f.write(f'        <inertial>\n')
        f.write(f'            <origin xyz="${{{name}_inertial_xyz}}" rpy="${{{name}_inertial_rpy}}" />\n')
        f.write(f'            <mass value="${{{name}_mass}}" />\n')
        f.write(f'            <inertia ixx="${{{name}_ixx}}" ixy="${{{name}_ixy}}" ixz="${{{name}_ixz}}" iyy="${{{name}_iyy}}" iyz="${{{name}_iyz}}" izz="${{{name}_izz}}" />\n')
        f.write(f'        </inertial>\n')
        f.write(f'    </link>\n')
        f.write(f'</xacro:macro>\n')
        f.write(f'\n')

    f.write(f'<xacro:macro name="finger_tip_link" params="prefix fingerprefix">\n')
    f.write(f'    <link name="${{prefix}}${{fingerprefix}}${{finger_tip_link_name}}">\n')
    f.write(f'        <visual>\n')
    f.write(f'            <origin xyz="${{finger_tip_visual_xyz}}" rpy="${{finger_tip_visual_rpy}}" />\n')
    f.write(f'            <geometry>\n')
    f.write(f'                <box size="${{finger_tip_box_dimension}}" />\n')
    f.write(f'            </geometry>\n')
    f.write(f'            <material name="${{finger_tip_visual_material_name}}">\n')
    f.write(f'                <color rgba="${{finger_tip_visual_material_color}}" />\n')
    f.write(f'            </material>\n')
    f.write(f'        </visual>\n')
    f.write(f'        <collision>\n')
    f.write(f'            <origin xyz="${{finger_tip_collision_xyz}}" rpy="${{finger_tip_collision_rpy}}" />\n')
    f.write(f'            <geometry>\n')
    f.write(f'                <box size="${{finger_tip_box_dimension}}" />\n')
    f.write(f'            </geometry>\n')
    f.write(f'            <material name="${{finger_tip_collision_material_name}}">\n')
    f.write(f'                <color rgba="${{finger_tip_collision_material_color}}" />\n')
    f.write(f'            </material>\n')
    f.write(f'        </collision>\n')
    f.write(f'    </link>\n')
    f.write(f'</xacro:macro>\n')
    f.write(f'\n')

    for name in link_names:
        f.write(f'<xacro:{name}_link prefix="${{prefix}}" fingerprefix="${{fingerprefix}}" />\n')

f.close()

# ---------------------------------------------------------------------------- #
#                                    JOINTS                                    #
# ---------------------------------------------------------------------------- #

# joint_configs = [
#     ['name', 'type', 'mimic'],
#     ['base_to_knuckle', 'revolute', False],
#     ['base_to_knuckle_mimic', 'revolute', True],
#     ['knuckle_to_bar', 'fixed', False],
#     ['bar_to_distal', 'revolute', True],
#     ['distal_to_tip', 'fixed', False],
# ]

# with open('joints.xacro', 'w') as f:
#     for config in joint_configs:
#         name, jtype, mimics = config

#     <xacro:macro name="outer_finger_joint" params="prefix fingerprefix">
#         <joint name="${prefix}${fingerprefix}_outer_finger_joint" type="fixed">
#             <origin xyz="0 0.0315 -0.0041" rpy="0 0 0"/>
#             <parent link="${prefix}${fingerprefix}_outer_knuckle" />
#             <child link="${prefix}${fingerprefix}_outer_finger" />
#             <axis xyz="1 0 0" />
#         </joint>
#     </xacro:macro>

# f.close()

# ---------------------------------------------------------------------------- #
#                                COMMON SECTIONS                               #
# ---------------------------------------------------------------------------- #

link_names = ['palm', 'knuckle', 'bar', 'proximal_phalanx', 'distal_phalanx']

with open('common.xacro', 'w') as f:
    for name in link_names:
        f.write(f'''<xacro:property name="{name}_mesh_files" value="${{sec_mesh_files['{name}']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_visual_parameters" value="${{{name}_mesh_files['visual']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_visual_xyz" value="${{{name}_visual_parameters['origin']['xyz']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_visual_rpy" value="${{{name}_visual_parameters['origin']['rpy']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_visual_mesh_file" value="${{{name}_visual_parameters['mesh']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_visual_mesh_scale" value="${{{name}_visual_parameters['scale']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_material_name" value="${{{name}_visual_parameters['material']['name']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_material_color" value="${{{name}_visual_parameters['material']['color']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_collision_parameters" value="${{{name}_mesh_files['collision']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_collision_xyz" value="${{{name}_collision_parameters['origin']['xyz']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_collision_rpy" value="${{{name}_collision_parameters['origin']['rpy']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_collision_mesh_file" value="${{{name}_collision_parameters['mesh']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_collision_mesh_scale" value="${{{name}_collision_parameters['scale']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_inertial_parameters" value="${{sec_inertial_params['{name}']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_inertial_xyz" value="${{{name}_inertial_parameters['origin']['xyz']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_inertial_rpy" value="${{{name}_inertial_parameters['origin']['rpy']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_mass" value="${{{name}_inertial_parameters['mass']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_ixx" value="${{{name}_inertial_parameters['inertia']['ixx']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_ixy" value="${{{name}_inertial_parameters['inertia']['ixy']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_ixz" value="${{{name}_inertial_parameters['inertia']['ixz']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_iyy" value="${{{name}_inertial_parameters['inertia']['iyy']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_iyz" value="${{{name}_inertial_parameters['inertia']['iyz']}}" scope="parent" />\n''')
        f.write(f'''<xacro:property name="{name}_izz" value="${{{name}_inertial_parameters['inertia']['izz']}}" scope="parent" />\n''')
        f.write(f'\n')

    f.write(f'''<xacro:property name="finger_tip_mesh_files" value="${{sec_mesh_files['finger_tip']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_box_dimension" value="${{finger_tip_mesh_files['box_dimensions']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_visual_parameters" value="${{finger_tip_mesh_files['visual']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_visual_xyz" value="${{finger_tip_visual_parameters['origin']['xyz']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_visual_rpy" value="${{finger_tip_visual_parameters['origin']['rpy']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_visual_material_name" value="${{finger_tip_visual_parameters['material']['name']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_visual_material_color" value="${{finger_tip_visual_parameters['material']['color']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_collision_parameters" value="${{finger_tip_mesh_files['collision']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_collision_xyz" value="${{finger_tip_collision_parameters['origin']['xyz']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_collision_rpy" value="${{finger_tip_collision_parameters['origin']['rpy']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_collision_material_name" value="${{finger_tip_collision_parameters['material']['name']}}" scope="parent" />\n''')
    f.write(f'''<xacro:property name="finger_tip_collision_material_color" value="${{finger_tip_collision_parameters['material']['color']}}" scope="parent" />\n''')

f.close()

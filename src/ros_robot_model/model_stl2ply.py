import trimesh

# 指定 STL 文件的路径
stl_file_path = './contactdb/water_bottle/water_bottle.stl'

# 指定 PLY 文件的输出路径
ply_file_path = './contactdb/water_bottle/water_bottle.ply'

# 加载 STL 文件
mesh = trimesh.load(stl_file_path)

# 保存为 PLY 文件
mesh.export(ply_file_path)

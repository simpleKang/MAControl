

# TODO 添加操作
def add(original, addlist):
    new = []
    return new


# TODO 插入操作
def insert(original, insertlist, pos):
    new = []
    return new


# TODO 删除操作
def delete(original, deleteindex):
    new = []
    return new


# TODO 攻击时替换列表
def attack_replace(original, coord, list_index):
    original.append([[0 for i in range(3)] for j in range(256)])
    list_index += 1
    original[list_index][0][0:3] = [coord[0], coord[1], 0]
    pointB_index = 0
    return original, list_index, pointB_index





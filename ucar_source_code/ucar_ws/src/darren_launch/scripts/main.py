# 这是一个示例 Python 脚本。

# 按 Shift+F10 执行或将其替换为您的代码。
# 按 双击 Shift 在所有地方搜索类、文件、工具窗口、操作和设置。
import random
import numpy


def random_draw():
    file = open("photo.txt", 'w').close()
    canteen = {'tableware ', 'person ', 'food ', 'chair '}
    bedroom = {'person ', 'pet ', 'bed ', 'chair '}
    living_room = {'sofa ', 'pet ', 'chair ', 'tv '}
    canteen_r = random.sample(canteen, 2) + random.sample(canteen, 1)
    bedroom_r = random.sample(bedroom, 2) + random.sample(bedroom, 1)
    living_room_r = random.sample(living_room, 2) + random.sample(living_room, 1)
    flag = random.randint(1, 6)
    with open("photo.txt", "w") as f:
        if flag == 1:
            f.writelines(canteen_r)
            f.write('\n')
            f.writelines(bedroom_r)
            f.write('\n')
            f.writelines(living_room_r)
            f.write('\n')
        if flag == 2:
            f.writelines(canteen_r)
            f.write('\n')
            f.writelines(living_room_r)
            f.write('\n')
            f.writelines(bedroom_r)
            f.write('\n')
        if flag == 3:
            f.writelines(bedroom_r)
            f.write('\n')
            f.writelines(canteen_r)
            f.write('\n')
            f.writelines(living_room_r)
            f.write('\n')
        if flag == 4:
            f.writelines(bedroom_r)
            f.write('\n')
            f.writelines(living_room_r)
            f.write('\n')
            f.writelines(canteen_r)
            f.write('\n')
        if flag == 5:
            f.writelines(living_room_r)
            f.write('\n')
            f.writelines(canteen_r)
            f.write('\n')
            f.writelines(bedroom_r)
            f.write('\n')
        if flag == 6:
            f.writelines(living_room_r)
            f.write('\n')
            f.writelines(bedroom_r)
            f.write('\n')
            f.writelines(canteen_r)
            f.write('\n')
    print(flag)


def function_judge(room):
    if {'food'}.issubset(room):
        result= '餐厅'
        find_flag=0
    if {'tableware'}.issubset(room):
        result= '餐厅'
        find_flag=0
    if {'bed'}.issubset(room):
        result= '卧室'
        find_flag=1
    if {'person', 'pet'}.issubset(room):
        result = '卧室'
        find_flag=1
    if {'tv'}.issubset(room):
        result = '客厅'
        find_flag=2
    if {'sofa'}.issubset(room):
        result = '客厅'
        find_flag=2
    return (result,find_flag)


def judge_room():
    # 在下面的代码行中使用断点来调试脚本。
    filename = 'photo.txt'
    result = ['', '', '']
    room_find_flag = [0, 0, 0]  # canteen,bedroom,livingroom
    with open(filename, 'r') as file:
        line = file.readline()
        photo1 = line.strip().split(' ')
        photo_1 = [photo1[0], photo1[1], photo1[2]]
        line = file.readline()
        photo2 = line.strip().split(' ')
        photo_2 = [photo2[0], photo2[1], photo2[2]]
        line = file.readline()
        photo3 = line.strip().split(' ')
        photo_3 = [photo3[0], photo3[1], photo3[2]]
        # for item in line:
        #     photo.append(item.strip('\n'))
        # photo=[photo1[0],photo1[1],photo1[2],photo2[0],photo2[1],photo2[2],photo3[0],photo3[1],photo3[2]]
    ## 判断B房间
    result[0],find_flag=function_judge(photo_1)
    room_find_flag[find_flag]=1
    ## 判断C房间
    result[1],find_flag=function_judge(photo_2)
    room_find_flag[find_flag]=1
    ## 判断D房间
    result[2],find_flag=function_judge(photo_3)
    room_find_flag[find_flag]=1
    try:
        f1 = result.index('')
        if room_find_flag.index(0) == 0:
            result[f1] = '餐厅'
        if room_find_flag.index(0) == 1:
            result[f1] = '卧室'
        if room_find_flag.index(0) == 2:
            result[f1] = '客厅'
        print(result)
    except:
        print(result)
    final_result = '任务完成,B房间为' + result[0] + ',C房间为' + result[1] + ',D房间为' + result[2] + '。'
    print(final_result)


if __name__ == '__main__':
    random_draw()
    judge_room()

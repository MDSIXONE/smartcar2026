def function_judge(room):
    result = ''
    find_index = -1
    if {'food'}.issubset(room):
        result= '餐厅'
        find_index=0
    if {'tableware'}.issubset(room):
        result= '餐厅'
        find_index=0
    if {'bed'}.issubset(room):
        result= '卧室'
        find_index=1
    if {'person', 'pet'}.issubset(room):
        result = '卧室'
        find_index=1
    if {'tv'}.issubset(room):
        result = '客厅'
        find_index=2
    if {'sofa'}.issubset(room):
        result = '客厅'
        find_index=2
    return result, find_index
def judge_room():
    # 在下面的代码行中使用断点来调试脚本。
    filename = '/home/ucar/ucar_ws/src/image/result.txt'
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
    ## 判断B房间
    result[0], find_index = function_judge(photo_1)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断C房间
    result[1], find_index = function_judge(photo_2)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断D房间
    result[2], find_index = function_judge(photo_3)
    if find_index != -1:
        room_find_flag[find_index] = 1
    try:
        f1 = result.index('')
        print(f1)
        if room_find_flag.index(0) == 0:
            result[f1] = '餐厅'
            # room_find_flag[0]=1
        if room_find_flag.index(0) == 1:
            result[f1] = '卧室'
            # room_find_flag[1]=1
        if room_find_flag.index(0) == 2:
            result[f1] = '客厅'
            # room_find_flag[2]=1
        print(result)
    except:
        print(result)
####
    # try:
    #     f1 = result.index('')
    #     print(f1)
    #     if room_find_flag.index(0) == 0:
    #         result[f1] = '餐厅'
    #         room_find_flag[0]=1
    #     if room_find_flag.index(0) == 1:
    #         result[f1] = '卧室'
    #         room_find_flag[1]=1
    #     if room_find_flag.index(0) == 2:
    #         result[f1] = '客厅'
    #         room_find_flag[2]=1
    #     print(result)
    # except:
    #     print(result)

    # try:
    #     f1 = result.index('')
    #     print(f1)
    #     if room_find_flag.index(0) == 0:
    #         result[f1] = '餐厅'
    #         room_find_flag[0]=1
    #     if room_find_flag.index(0) == 1:
    #         result[f1] = '卧室'
    #         room_find_flag[1]=1
    #     if room_find_flag.index(0) == 2:
    #         result[f1] = '客厅'
    #         room_find_flag[2]=1
    #     print(result)
    # except:
    #     print(result)
###
    final_result = '任务完成,B房间为' + result[0] + ',C房间为' + result[1] + ',D房间为' + result[2] + '。'
    print(final_result)
    return final_result

if __name__ == '__main__':
    judge_room()

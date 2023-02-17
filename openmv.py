import sensor, image, time, os, tf, math,machine
from pyb import UART
import json
from pyb import LED
import lcd

sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 240))       # Set 240x240 window.
sensor.set_auto_whitebal(False, rgb_gain_db=(65.2256,60.2071,61.9736))
sensor.set_auto_exposure(False, 60000)
sensor.set_auto_gain(False, 22)
#sensor.set_auto_whitebal(False)
#sensor.set_auto_exposure(False)
#sensor.set_auto_gain(False)
sensor.skip_frames(time=2000)          # Let the camera adjust.
clock = time.clock()

#13-14 45
thresholds = (0, 60, -40, -20, 0, 20)#非场地颜色
#thresholds = (30, 60, -60, -20, 5, 40)#非场地颜色
thresholds4 = (30, 75, -60, -20, 5, 50)#非场地颜色
thresholds3 = (0, 75, -80, -20, 5, 80)#非场地颜色
thresholds2 = (30, 60, -80, -20, 0, 80)#中心颜色是不是绿色剔除跳动

red_lab=(0,100,40,70,20,60)#红色堆放区
blue_lab=(75,100,-40,-10,-30,0)#蓝色堆放区
green_lab=(40,70,-60,-35,20,60)#绿色堆放区
hui_lab=(45,75,-20,0,5,30)#灰色堆放区
#black_lab=(0,35,-35,5,-5,35)#黑色队伍
#black_lab=(0,30,-30,5,-20,30)#黑色队伍
black_lab=(0,25,-25,5,-5,25)#黑色队伍
yellow_lab=(60,100,-40,0,40,80)#黄色队伍
wlljw_cup_lab=(45,75,-30,10,5,35)
wlljw_battery_lab=(30,75,-20,40,0,60)
j1s_lab=(30,60,-40,0,0,40)
j2s_lab=(40,50,-40,-20,0,20)
jb_lab=(0,45,-10,-30,0,30)
j_lab=(0,100,-40,0,0,40)
jg_lab=(30,60,-40,-10,0,30)
jw_lab=(50,75,-30,-20,0,20)
write_lab=(90,100,-20,5,-5,20)#白线
spitball_lab=(45,75,-20,5,-10,10)#白线上的纸团
rub_lab=(0,100,-70,-20,20,60)#垃圾
red = (0, 100, 10, 60, -5, 50)#红色堆放区
paper_lab=(0,100,-30,0,0,30)
orange_lab=((0,100,-10,40,40,80))


#串口通信
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)  #8位数据位，无校验位，1位停止位

#返回最大色块（也就是返回最近的垃圾）
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob
#垃圾识别
def judge(pix):
    if pix>150 and pix<1000:return 'battery'#5000-14000
    if pix>=1000 and pix<=1500:return 'orange'#400-1310
    if pix>=1500 and pix<=5000:return 'none'#400-1310
    if pix>=6000 and pix<=25000:return 'cup'#400-1310

#参数--------------------------------------------------------
#堆放区参数---------
red_h=0;red_cx=0
black_x=0;black_y=1;black_h=3;black_w=1
black_jh=1;black_jw=1;black_jy=0;black_sh=1
yellow_x=0;yellow_y=1;yellow_h=1;yellow_w=1
yellow_jh=0;yellow_jw=0;yellow_jy=0;yellow_sh=1
find_dui=0;find_2=0;find_3=0#发送堆放区的代码100，第二位代表，第三位代表是否发现堆放区
frist_3=0;frist_2=0;five_3=0;five_2=0#用来纠偏
black_g_cx=240;black_b_cx=240;black_r_cx=240;black_h_cx=240;
black_lsy=0;black_lsx2=0
yellow_g_cx=240;yellow_b_cx=240;yellow_r_cx=240;yellow_h_cx=240;
yellow_lsy=0;yellow_lsx2=0;
b_b_h=0;b_a_h=0;y_a_h=0;y_b_h=0
hui_x2=0
blue_x=0;hui_x=240#用来限制堆放区的左右
dui_cx=0#当前要找的堆放区的x坐标
need='2'#用来判断传哪一个堆放区的(蓝2，绿1，红3，灰4)
team=2#用来判断我是哪个队伍的（1黑色。2黄色）
run=3#用来判断是执行那一部分的代码（1找垃圾。2微调。3找识别垃圾.4找堆放区）
#垃圾识别参数---------
x=5;y=5;w1=1;h1=1
cx=0;cy=0;cw=1;ch=1
count_2=0;max_2x=[0,0];rub_cx2=0;rub_cy2=0
jbbb_x=0;jbbb_y=0;jbbb_w=1;jbbb_h=1;
bi_zhuang_x=0;g_all_area=0


#lcd.init() # Initialize the lcd screen.

while(True):
    #print(clock.fps())
    #print(sensor.get_exposure_us())#当前摄像头曝光时间
    #print(sensor.get_rgb_gain_db())#返回当前摄像机红色，绿色和蓝色增益值以分贝((浮点型，浮点型，浮点型))表示的元组。
    clock.tick()
    img = sensor.snapshot()
    #LED(3).on()
    LED(2).on()
    #lcd.display(img,roi=(56,40,128,160))
    #print(lcd.width(),lcd.height())
    #数据接收与处理--------------------------------------------------------
    car_data=uart.read(1)
    #print(car_data)
    if car_data!=b'1' and car_data!=b'2' and car_data!=b'3' and car_data!=b'4' and car_data!=b'5' and car_data!=b'6' and car_data!=b'7' and car_data!=b'8':
        #run=1;team=2
        pass
    else:
        #1-4黑色区域
        if car_data==b'1':run=1#运行找垃圾的代码
        if car_data==b'2':run=2#运行微调代码
        if car_data==b'3':run=3#运行识别垃圾的代码
        if car_data==b'4':run=4#运行找堆放区的代码
        #5-8黄色区域
        if car_data==b'5':run=5#运行找垃圾的代码
        if car_data==b'6':run=6#运行微调代码
        if car_data==b'7':run=7#运行识别垃圾的代码
        if car_data==b'8':run=8#运行找堆放区的代码

    #找垃圾----------------------------------------------------------------------------
    if run==1 or run==5 or run==2 or run==6:
        sum_y=0;sum_x=0;count_y=0;count_x=0
        count_40=0#计数满20
        rub_cx=0;rub_cy=0
        size_x=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#40位
        size_y=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#40位
        areas=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#40位
        r=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#40位
        #得到限制的色块---------------------------------------------------
        for i in img.find_blobs([thresholds], pixels_threshold=20, area_threshold=20, invert=True):
            n=0
            x=i.x();y=i.y();w1=i.w();h1=i.h()#上下左右颜色判断
            jw=min(w1,h1);jh=max(w1,h1)
            if y-5<=0:ys=5;#上
            else:ys=y
            if y+h1+5>=240:yx=240-h1-5#下
            else:yx=y
            if x-5<=0:x1=5#左
            else:x1=x
            if x+w1+5>=240:x2=240-w1-5#左
            else:x2=x
            cw=(int)(w1/3);ch=(int)(h1/3);cx=x+cw;cy=y+ch#中心颜色判断
            if cw==0:cw=1
            if ch==0:ch=1
            if ch*cw<15:cw=w1;ch=h1
            if cx>240:cx=240
            if cy>240:cy=240
            bian_x=i.x()-5;bian_y=i.y()-5;bian_w=i.w()+10;bian_h=i.h()+10#边缘检测
            if bian_x<0:bian_x=0
            if bian_y<0:bian_y=0
            if bian_w+bian_x>240:bian_w=i.w()+5
            if bian_h+bian_y>240:bian_h=i.h()+5
            jbbb_x=i.x()-40;jbbb_y=i.y();jbbb_w=40;jbbb_h=40;#排除绿色堆放区的垃圾
            if jbbb_y<0:jbbb_y=i.y();
            if jbbb_x<0:jbbb_x=i.x();
            if jbbb_x+jbbb_w>240:jbbb_w=2;
            if jbbb_y+jbbb_h>240:jbbb_h=2;
            g=img.find_blobs([thresholds2],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            #g_all=img.find_blobs([thresholds],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15)#根据绿色占比提出杂色
            #for j in g_all:g_all_area=g_all_area+j.pixels()#根据绿色占比提出杂色
            #g_zhan_bi=g_all_area/i.area()#根据绿色占比提出杂色
            #g_all_area=0
            js=img.find_blobs([j_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)

            jbbbs=img.find_blobs([blue_lab],roi=(0,jbbb_y,240,jbbb_h), pixels_threshold=15, area_threshold=15)
            jbb_j=1;
            if jbbbs:
                max_jbs=find_max(jbbbs);img.draw_rectangle(max_jbs.rect(),color=(0,0,0),thickness=2)
                if max_jbs.y()+max_jbs.h()+10>i.y() and max_jbs.x()<i.x():jbb_j=0;
                else:pass
            else:jbb_j=1;
            #jy=img.find_blobs([yellow_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            jy=1
            j1s=img.find_blobs([j1s_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            j2s=img.find_blobs([j2s_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            jg=img.find_blobs([jg_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            jb=img.find_blobs([jb_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            jwr=img.find_blobs([jw_lab],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            gs = img.find_blobs([thresholds4],roi=(x,ys-5,w1,5), pixels_threshold=10, area_threshold=10)
            gx = img.find_blobs([thresholds4],roi=(x,yx+h1,w1,5),pixels_threshold=10, area_threshold=10)
            gz = img.find_blobs([thresholds4],roi=(x1-5,y,5,h1),pixels_threshold=10, area_threshold=10)
            gy = img.find_blobs([thresholds4],roi=(x2+w1,y,5,h1),pixels_threshold=10, area_threshold=10)
            #img.morph(kernel_size, kernel)
            #img.binary(thresholds3)
            #bian=img.erode(1, threshold = 2)
            #b=bian.find_blobs([white],roi=(bian_x,bian_y,bian_w,bian_h), pixels_threshold=5, area_threshold=5,merge=True)#用边缘检测看是否为错误识别
            #img.draw_rectangle((cx,cy,cw,ch),color=(225,0,0),thickness=2)
            #img.draw_rectangle((x,ys-5,w1,5),color=(225,0,0),thickness=2)
            #img.draw_rectangle((x,yx+h1,w1,5),color=(225,0,0),thickness=2)
            #img.draw_rectangle((x1-5,y,5,h1),color=(225,0,0),thickness=2)
            #img.draw_rectangle((x2+w1,y,5,h1),color=(225,0,0),thickness=2)
            if gs:n=n+1
            if gx:n=n+1
            if gz:n=n+1
            if gy:n=n+1
            img.draw_rectangle(i.rect(),color=(0,0,0),thickness=2)
            #img.draw_rectangle((bian_x,bian_y,bian_w,bian_h),color=(0,225,0),thickness=2)
            #if g:print('---------')
            #if b:print('=========')
            #找垃圾
            if run==1 or run==5:
                #普通垃圾
                if n>=2 and w1<100 and h1<100 and g and jb and jbb_j  and jwr and jw/jh>0.2 and y>20 and i.pixels()<=4250 and i.y()+i.h()!=240 and i.x()>20 and i.x()+i.w()<220  and i.y()>20 and i.y()+i.h()<220 and i.x()+i.w()!=240:
                #if n<=3 and w1<100 and h1<100 and g and y>20 and i.pixels()<=4250 and i.y()+i.h()!=240 and i.x()>20 and i.x()<220  and i.y()>20 and i.y()<220 and i.x()+i.w()!=240:
                #if w1<100 and h1<100 and b and jw/jh>0.2 and y>20 and i.pixels()<=4250 and i.y()+i.h()!=240 and i.x()>20 and i.x()<220  and i.y()<210 and i.x()+i.w()!=240:
                    if (i.y()+i.h()>=0 and i.y()+i.h()<=80 and i.pixels()<=1700 and jy) or (i.y()+i.h()>80 and i.y()+i.h()<=160 and i.pixels()<=3000 and jy) or (i.y()+i.h()>=160 and i.y()+i.h()<=240 and i.pixels()<=4250 and i.area()>=150 and jy) :
                        img.draw_rectangle(i.rect(),color=(225,0,0),thickness=2)
                        img.draw_string(i.x(),i.y()-10,'find', color=(225,0,0))
                        size_x[count_40]=i.cx();size_y[count_40]=i.cy();count_40=count_40+1#获取每一个色块的cx，cy
                        #print(g_zhan_bi)
                        #print(i.density())
                ##白线上的垃圾
                #else:
                    #wlljw=img.find_blobs([write_lab],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15)
                    #white_line=img.find_blobs([orange_lab,spitball_lab,wlljw_battery_lab],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15,merge=True)
                    #if i.area()>4000:
                        #for wljj in white_line:
                            #wljj_w=wljj.w();wljj_h=wljj.h();wljj_wh=min(wljj_w,wljj_h)/max(wljj_w,wljj_h)
                            ##img.draw_rectangle(wljj.rect(),color=(0,225,0),thickness=2)
                            #if wljj.area()>100 and wljj.area()<3700 and wlljw  and wljj_wh>0.2 and wljj.y()>80 and wljj.y()+wljj.h()<220 and wljj.x()>20 and wljj.x()+wljj.w()<220  and wljj.y()>20 and wljj.y()<220:
                                #img.draw_rectangle(wljj.rect(),color=(0,0,225),thickness=2)
                                #img.draw_string(wljj.x(),wljj.y()-10,'find', color=(0,0,225))
                                #size_x[count_40]=wljj.cx();size_y[count_40]=wljj.cy();count_40=count_40+1#获取每一个色块的cx，cy
            #微调
            if run==2 or run==6:
                #普通垃圾
                if n>=2 and w1<200 and h1<200 and g and jb and jwr and jw/jh>0.2 and i.area()>400:
                    #if (i.y()+i.h()>=0 and i.y()+i.h()<=80 and i.area()>100) or (i.y()+i.h()>80 and i.y()+i.h()<=160 and i.area()>=200 and jy) or (i.y()+i.h()>=160 and i.y()+i.h()<=240 and i.area()>=400) :
                    #print(i.density())锁定效果
                    img.draw_rectangle(i.rect(),color=(225,0,0),thickness=2)
                    img.draw_cross(i.cx(), i.cy(), size=5, color=(225,0,0))
                    img.draw_string(i.cx(), i.cy(),'(%d,%d)'%(i.cx(), i.cy()), color=(225,0,0))
                    img.draw_line((120,120,i.cx(), i.cy()), color=(225,0,0))
                    img.draw_cross(120,120, size=5, color=(225,0,0))
                    img.draw_string(120,120,'(%d,%d)'%(120,120), color=(225,0,0))
                    size_x[count_40]=i.cx();size_y[count_40]=i.cy();areas[count_40]=i.area();count_40=count_40+1#获取每一个色块的cx，cy

                ###白线上的微调
                #else:
                    ##print('---------------------------------')
                    #wlljw=img.find_blobs([write_lab],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15)
                    #white_line=img.find_blobs([orange_lab,spitball_lab,wlljw_battery_lab],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15,merge=True)
                    #if i.area()>4000:
                        #for wljj in white_line:
                            #wljj_w=wljj.w();wljj_h=wljj.h();wljj_wh=min(wljj_w,wljj_h)/max(wljj_w,wljj_h)
                            ##img.draw_rectangle(wljj.rect(),color=(0,225,0),thickness=2)
                            #if wljj.area()>100 and wlljw  and wljj_wh>0.2:
                                #img.draw_rectangle(wljj.rect(),color=(0,0,225),thickness=2)
                                #img.draw_string(wljj.cx(), wljj.cy(),'(%d,%d)'%(wljj.cx(), wljj.cy()), color=(0,0,225))
                                #img.draw_line((120,120,wljj.cx(), wljj.cy()), color=(0,0,225))
                                #img.draw_cross(120,120, size=5, color=(0,0,225))
                                #img.draw_string(120,120,'(%d,%d)'%(120,120), color=(0,0,225))
                                #size_x[count_40]=wljj.cx();size_y[count_40]=wljj.cy();areas[count_40]=wljj.area();count_40=count_40+1#获取每一个色块的cx，cy
                #print(jw/jh)
                #print(i.pixels())#距离40px
        #参数获取与传输---------------------------------------------
        max_y=max(size_y[0],size_y[1],size_y[2],size_y[3],size_y[4],size_y[5],size_y[6],size_y[7],size_y[8],size_y[9],size_y[10],\
        size_y[11],size_y[12],size_y[13],size_y[14],size_y[15],size_y[16],size_y[17],size_y[18],size_y[19],size_y[20],size_y[21],\
        size_y[22],size_y[23],size_y[24],size_y[25],size_y[26],size_y[27],size_y[28],size_y[29],size_y[30],size_y[31],size_y[32],\
        size_y[33],size_y[34],size_y[35],size_y[36],size_y[37],size_y[38],size_y[39])
        #获取微调坐标(水瓶求平均值)
        #if run==2 or run==6:
            #for i in range(40):
                #if size_y[i]!=0:sum_y=sum_y+size_y[i];count_y=count_y+1
                #if count_y==0:sum_y=0;count_y=40
            #avg_y=sum_y/count_y
            #i=0
            #for i in range(40):
                #if size_x[i]!=0:sum_x=sum_x+size_x[i];count_x=count_x+1
                #if count_x==0:sum_x=0;count_x=40
            #avg_x=sum_x/count_x
        #获取微调坐标2(半径最小)
        if run==2 or run==6:
            for i in range(40):
                r[i]=(size_x[i]-120)*(size_x[i]-120)+(size_y[i]-120)*(size_y[i]-120)
            min_r=min(r)
            for i in range(40):
                if r[i]==min_r:avg_y=size_y[i];avg_x=size_x[i]

        #获取最近的色块
        count_40=0
        for count_40 in range(40):
            if max_y==size_y[count_40] and max_y!=0:
                #print(size_x[count_40],max_y);
                break;
        #获取2次x坐标，剔除跳动
        max_2x[count_2]=size_x[count_40]
        if abs(max_2x[0]-max_2x[1])<=30 and max_y!=0:
            rub_cx=size_x[count_40];rub_cy=max_y#剔除跳动
            rub_cx2=rub_cx;rub_cy2=rub_cy#保留数据
        else:
            if max_y!=0:#确保视野内是有色块的
                rub_cx=rub_cx2;rub_cy=rub_cy2#给上一次的数据
            #print(abs(max_2x[0]-max_2x[1]))#用来去除跳动
        #数据传输
        if run==1 or run==5:
            uart.write("%d%d%d%d%d%d%d"%(111,999,999,999,999,rub_cx+100,rub_cy+100))#yes=101视野内有色块,match=101匹配，可以投放
            print("%d%d%d%d%d%d%d"%(111,999,999,999,999,rub_cx+100,rub_cy+100))
        if run==2 or run==6:
            uart.write("%d%d%d%d%d%d%d"%(222,999,999,999,999,avg_x+100,avg_y+100))#yes=101视野内有色块,match=101匹配，可以投放
            print("%d%d%d%d%d%d%d"%(222,999,999,999,999,avg_x+100,avg_y+100))
        rub_cx=0;rub_cy=0
        if count_2==1:max_2x=[0,0];count_2=0
        else:count_2=count_2+1

    #识别垃圾------------------------------------------------------------------------------
    if run==3 or run==7:
        #-------------------------------------找垃圾--------------------------------------------------
        #框选垃圾
        j_water=0;count=0;oranges=0
        pix=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#40位
        jhw=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#40位
        #得到限制的色块---------------------------------------------------
        for i in img.find_blobs([thresholds], pixels_threshold=20, area_threshold=20, invert=True):
            n=0
            x=i.x();y=i.y();w1=i.w();h1=i.h()#上下左右颜色判断
            jw=max(i.w(),i.h());jh=min(i.w(),i.h())
            cw=(int)(w1/3);ch=(int)(h1/3);cx=x+cw;cy=y+ch#中心颜色判断
            if cw==0:cw=1
            if ch==0:ch=1
            if ch*cw<15:cw=w1;ch=h1
            if cx>240:cx=240
            if cy>240:cy=240
            g=img.find_blobs([thresholds2],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            orang=img.find_blobs([orange_lab],roi=i.rect(), pixels_threshold=15, area_threshold=15)
            if orang:oranges=oranges+1
            #微调
            if run==3 or run==7:
                #普通垃圾
                if w1<200 and h1<200 and g and i.y()+i.h()!=240 and i.cx()>20 and i.cx()<220  and i.cy()<210 and i.cy()>20 and i.x()+i.w()!=240:
                    img.draw_rectangle(i.rect(),color=(225,0,0),thickness=2)
                    j_water=j_water+1
                    pix[count]=i.pixels();jhw[count]=jh/jw;count=count+1

                ###白线上的垃圾
                #else:
                    #img.draw_rectangle(i.rect(),color=(0,0,0),thickness=2)
                    #wlljw=img.find_blobs([write_lab],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15)
                    #white_line=img.find_blobs([orange_lab,spitball_lab,wlljw_battery_lab],roi=(i.x(),i.y(),i.w(),i.h()), pixels_threshold=15, area_threshold=15,merge=True)
                    #if i.area()>4000:
                        #for wljj in white_line:
                            #wljj_w=wljj.w();wljj_h=wljj.h();wljj_wh=min(wljj_w,wljj_h)/max(wljj_w,wljj_h)
                            ##img.draw_rectangle(wljj.rect(),color=(0,225,0),thickness=2)
                            #if wljj.area()>100  and wlljw  and wljj_wh>0.2 and wljj.y()>20 and wljj.y()+wljj.h()<220 and wljj.x()>20 and wljj.x()+wljj.w()<220  and wljj.y()>20 and wljj.y()<220:
                                #img.draw_rectangle(wljj.rect(),color=(0,0,225),thickness=2)
                                #j_water=j_water+1
                                #pix[count]=wljj.pixels();jhw[count]=jh/jw;count=count+1
        count=0
        for count in range(40):
            if pix[count]==max(pix):break
        #print(j_water)#>=4为水瓶
        #print(jh/jw)
        #print(max(pix))#距离40px >6000 >1500 <5000 >1000 <5000   >300 1000<
        blos=img.find_blobs([thresholds], pixels_threshold=20, area_threshold=20, invert=True)
        if blos:max_blo=find_max(blos)
        if j_water>=4:
            print('water')
            img.draw_string(max_blo.x(),max_blo.y()-20,'water', color=(225,0,0),scale=2)
        else:
            #print(max(pix))
            if judge(max(pix))=='cup':
                print('cup');
                need='4';
                img.draw_string(max_blo.x(),max_blo.y()-20,'cup', color=(225,0,0),scale=2)
            if judge(max(pix))=='battery':
                print('battery');
                need='3';
                img.draw_string(max_blo.x(),max_blo.y()-20,'battery', color=(225,0,0),scale=2)
            if judge(max(pix))=='orange':
                print('orange');
                need='1';
                img.draw_string(max_blo.x(),max_blo.y()-20,'orange', color=(225,0,0),scale=2)
            if judge(max(pix))=='none':
                if oranges:
                    print('orange');
                    need='1';
                    img.draw_string(max_blo.x(),max_blo.y()-20,'orange', color=(225,0,0),scale=2)
                elif jhw[count]>=0.8:
                    print('paper');
                    need='2';
                    img.draw_string(max_blo.x(),max_blo.y()-20,'spitball', color=(225,0,0),scale=2)
                else:
                    print('none');
                    need='2';
                    img.draw_string(max_blo.x(),max_blo.y()-20,'none', color=(225,0,0),scale=2)

    #找堆放区------------------------------------------------------
    if run==4 or run==8:
        #全场地寻找色块+++++++++++++++++++++++
        red_blobs = img.find_blobs([red_lab], pixels_threshold=200, area_threshold=200)#找红色堆放区
        black_blobs = img.find_blobs([black_lab], pixels_threshold=200, area_threshold=200,merge=True)#找黑色小队
        yellow_blobs = img.find_blobs([yellow_lab], pixels_threshold=200, area_threshold=200,merge=True)#找黄色小队
        rub_2_blobs=img.find_blobs([thresholds], roi=(0,100,240,140),pixels_threshold=200, area_threshold=200,merge=True,invert=True)#避障
        ##用于避障
        #for i in img.find_blobs([thresholds], pixels_threshold=20, area_threshold=20, invert=True):
            #n=0
            #x=i.x();y=i.y();w1=i.w();h1=i.h()#上下左右颜色判断
            #jw=max(i.w(),i.h());jh=min(i.w(),i.h())
            #cw=(int)(w1/3);ch=(int)(h1/3);cx=x+cw;cy=y+ch#中心颜色判断
            #if cw==0:cw=1
            #if ch==0:ch=1
            #if ch*cw<15:cw=w1;ch=h1
            #if cx>240:cx=240
            #if cy>240:cy=240
            #g=img.find_blobs([thresholds2],roi=(cx,cy,cw,ch), pixels_threshold=15, area_threshold=15,invert=True)
            #for max_2 in rub_2_blobs:
                ##max_2=find_max(rub_2_blobs)
                #if max_2.pixels()<5000 and g and min(max_2.w(),max_2.h())/max(max_2.w(),max_2.h())>0.2 and max_2.y()<210 and max_2.x()!=0 and max_2.x()+max_2.w()!=240 and max_2.y()+max_2.h()!=240 and max_2.w()<100:
                    #img.draw_rectangle(max_2.rect(),color=(225,0,0),thickness=2)
                    #bi_zhuang_x=max_2.cx()
        #限制区域寻找色块+++++++++++++++++++
        #黑色上的-红，绿，蓝，灰
        j_black_gblobs = img.find_blobs([green_lab],roi=(black_x,black_y,black_w,black_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找绿色堆放区
        j_black_bblobs = img.find_blobs([blue_lab],roi=(black_x,black_y,black_w,black_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找蓝色堆放区
        j_black_rblobs = img.find_blobs([red_lab],roi=(black_x,black_y,black_w,black_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找红色堆放区
        j_black_hblobs = img.find_blobs([hui_lab],roi=(black_x,black_y,black_w,black_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找红色堆放区
        #黄色上的-红，绿，蓝，灰
        j_yellow_gblobs = img.find_blobs([green_lab],roi=(yellow_x,yellow_y,yellow_w,yellow_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找绿色堆放区
        j_yellow_bblobs = img.find_blobs([blue_lab],roi=(yellow_x,yellow_y,yellow_w,yellow_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找蓝色堆放区
        j_yellow_rblobs = img.find_blobs([red_lab],roi=(yellow_x,yellow_y,yellow_w,yellow_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找红色堆放区
        j_yellow_hblobs = img.find_blobs([hui_lab],roi=(yellow_x,yellow_y,yellow_w,yellow_h) ,pixels_threshold=200, area_threshold=200,merge=True)#找红色堆放区
        #用于调偏移的色块+++++++++++++++++++
        #黑--下的-左右非黑-充满底部的
        lx_b_blobs = img.find_blobs([black_lab],roi=(0,200,40,40), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        rx_b_blobs = img.find_blobs([black_lab],roi=(200,200,40,40), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        #黑--上-下的-左右非黑-随黑框变化
        lsx_b_blobs = img.find_blobs([black_lab],roi=(black_x,black_lsy,30,30), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        rsx_b_blobs = img.find_blobs([black_lab],roi=(black_lsx2,black_lsy,30,30), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        #黑--上-上的-左右非黑-随黑框变化
        rx_b_blobs = img.find_blobs([black_lab],roi=(black_x,black_jy,40,black_sh), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        lx_b_blobs = img.find_blobs([black_lab],roi=(black_x+black_w-40,black_jy,40,black_sh), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        #黄--下的-左右非黄-充满底部的
        lx_y_blobs = img.find_blobs([yellow_lab],roi=(0,200,40,40), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        rx_y_blobs = img.find_blobs([yellow_lab],roi=(200,200,40,40), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        #黄--上-下的-左右非黄-随黄框变化
        lsx_y_blobs = img.find_blobs([yellow_lab],roi=(yellow_x,yellow_lsy,30,30), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        rsx_y_blobs = img.find_blobs([yellow_lab],roi=(yellow_lsx2,yellow_lsy,30,30), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        #黄--上-上的-左右非黄-随黄框变化
        rx_y_blobs = img.find_blobs([yellow_lab],roi=(yellow_x,yellow_jy,40,yellow_sh), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        lx_y_blobs = img.find_blobs([yellow_lab],roi=(yellow_x+yellow_w-40,yellow_jy,40,yellow_sh), pixels_threshold=200, area_threshold=200,merge=True,invert=True)#找黑色小队
        #堆放区域参数获取------------------------------------------------------------
        #框出红色堆放区
        if red_blobs:
            red_blob = find_max(red_blobs)
            img.draw_rectangle(red_blob.rect(),color=(225,0,0),thickness=2)
            red_cx=red_blob.cx()
        #框出黑色队伍
        if black_blobs:
            black_blob = find_max(black_blobs)
            img.draw_rectangle(black_blob.rect(),color=(0,0,0),thickness=2)
            black_h=abs((int)(black_blob.h()*2/3))#用于限制识别
            black_x=black_blob.x();black_w=black_blob.w()
            black_jh=black_blob.h();black_jw=black_blob.w();black_jy=black_blob.y();
            black_y=black_blob.y()-black_blob.h();
            if black_y<0:black_y=0
            black_lsy=black_jy+black_jh-30
            if black_lsy<0:black_lsy=0
            black_lsx2=black_x+black_w-30
            if black_lsx2<0:black_lsx2=0
            black_sh=(int)(black_blob.h()/2)
        #框出黄色队伍
        if yellow_blobs:
            yellow_blob = find_max(yellow_blobs)
            img.draw_rectangle(yellow_blob.rect(),color=(225,225,0),thickness=2)
            yellow_h=abs((int)(yellow_blob.h()*2/3))#用于限制识别
            yellow_x=yellow_blob.x();yellow_w=yellow_blob.w()
            yellow_jh=yellow_blob.h();yellow_jw=yellow_blob.w();yellow_jy=yellow_blob.y();
            yellow_y=yellow_blob.y()-yellow_blob.h();
            if yellow_y<0:yellow_y=0
            yellow_lsy=yellow_jy+yellow_jh-30
            if yellow_lsy<0:yellow_lsy=0
            yellow_lsx2=yellow_x+yellow_w-30
            if yellow_lsx2<0:yellow_lsx2=0
            yellow_sh=(int)(yellow_blob.h()/2)
        #框出-黑色-限制区域-----------------------------------------------------------------
        if run==4:
            if black_blobs and j_black_gblobs:#黑色区域，并且黑色上面有绿色
                j_black_gblob = find_max(j_black_gblobs)
                black_max=find_max(black_blobs)
                img.draw_rectangle(j_black_gblob.rect(),color=(0,225,0),thickness=2)
                if need=='1' and black_max.pixels()>=7000:dui_cx=j_black_gblob.cx()#去哪个区域
                black_g_cx=abs(j_black_gblob.cx()-120)
            if black_blobs and j_black_bblobs:#黑色区域，并且黑色上面有蓝色
                j_black_bblob = find_max(j_black_bblobs)
                black_max=find_max(black_blobs)
                blue_x=j_black_bblob.x()
                img.draw_rectangle(j_black_bblob.rect(),color=(0,0,225),thickness=2)
                if need=='2' and black_max.pixels()>=7000:dui_cx=j_black_bblob.cx();#去哪个区域
                black_b_cx=abs(j_black_bblob.cx()-120)
            if black_blobs and j_black_rblobs:#黑色区域，并且黑色上面有红色
                j_black_rblob = find_max(j_black_rblobs)
                black_max=find_max(black_blobs)#
                img.draw_rectangle(j_black_rblob.rect(),color=(225,0,0),thickness=2)
                if need=='3' and black_max.pixels()>=7000:dui_cx=j_black_rblob.cx();#去哪个区域
                black_r_cx=abs(j_black_rblob.cx()-120)
            if black_blobs and j_black_hblobs:#黑色区域，并且黑色上面有灰色
                j_black_hblob = find_max(j_black_hblobs)
                black_max=find_max(black_blobs)
                img.draw_rectangle(j_black_hblob.rect(),color=(128,128,128),thickness=2)
                if need=='4' and black_max.pixels()>=7000:dui_cx=j_black_hblob.cx();#去哪个区域
                black_h_cx=abs(j_black_hblob.cx()-120)
                hui_x=j_black_hblob.x()+j_black_hblob.w();hui_x2=j_black_hblob.x()

        #框出-黄色-限制区域-----------------------------------------------------------------
        if run==8:
            if yellow_blobs and j_yellow_gblobs:#黄色区域，并且黄色上面有绿色
                j_yellow_gblob = find_max(j_yellow_gblobs)
                yellow_max=find_max(yellow_blobs)
                img.draw_rectangle(j_yellow_gblob.rect(),color=(0,225,0),thickness=2)
                if need=='1' and yellow_max.pixels()>=7000:dui_cx=j_yellow_gblob.cx()#去哪个区域
                yellow_g_cx=abs(j_yellow_gblob.cx()-120)
            if yellow_blobs and j_yellow_bblobs:#黄色区域，并且黄色上面有蓝色
                j_yellow_bblob = find_max(j_yellow_bblobs)
                yellow_max=find_max(yellow_blobs)
                img.draw_rectangle(j_yellow_bblob.rect(),color=(0,0,225),thickness=2)
                blue_x=j_yellow_bblob.x()
                if need=='2' and yellow_max.pixels()>=7000:dui_cx=j_yellow_bblob.cx();#去哪个区域
                yellow_b_cx=abs(j_yellow_bblob.cx()-120)
            if yellow_blobs and j_yellow_rblobs:#黄色区域，并且黄色上面有红色
                j_yellow_rblob = find_max(j_yellow_rblobs)
                yellow_max=find_max(yellow_blobs)
                img.draw_rectangle(j_yellow_rblob.rect(),color=(225,0,0),thickness=2)
                if need=='3' and yellow_max.pixels()>=7000:dui_cx=j_yellow_rblob.cx();#去哪个区域
                yellow_r_cx=abs(j_yellow_rblob.cx()-120)
            if yellow_blobs and j_yellow_hblobs:#黑色区域，并且黑色上面有灰色
                j_yellow_hblob = find_max(j_yellow_hblobs)
                yellow_max=find_max(yellow_blobs)
                img.draw_rectangle(j_yellow_hblob.rect(),color=(128,128,128),thickness=2)
                if need=='4' and yellow_max.pixels()>=7000:dui_cx=j_yellow_hblob.cx();#去哪个区域
                yellow_h_cx=abs(j_yellow_hblob.cx()-120)
                hui_x=j_yellow_hblob.x()+j_yellow_hblob.w();hui_x2=j_yellow_hblob.x()
        #堆放区域判断----------------------------------------------------------------
        #红色x坐标判断是否为堆放区
        if yellow_blobs or black_blobs:#判断是否停下--黑色/红色充满底部
            if yellow_jh+yellow_jy==240 and yellow_jw>=120:find_3=1
            if black_jh+black_jy==240 and black_jw>=120:find_3=2#极端情况不行
        else:find_3=0
        #判断下部是否偏移
        if find_3==2:#黑色下部分是否偏移
            if lx_b_blobs:frist_3=1
            if rx_b_blobs:frist_3=2
            if lx_b_blobs and rx_b_blobs:frist_3=0
        if find_3==1:#黄色下部分是否偏移
            if lx_y_blobs:frist_3=1
            if rx_y_blobs:frist_3=2
            if lx_y_blobs and rx_y_blobs:frist_3=0
        #判断-黑色-上部是否偏移
        if run==4:
            if lsx_b_blobs:img.draw_rectangle(find_max(lsx_b_blobs).rect(),color=(225,225,225),thickness=2)
            if rsx_b_blobs:img.draw_rectangle(find_max(rsx_b_blobs).rect(),color=(225,225,225),thickness=2)
            if rx_b_blobs:
                a=find_max(rx_b_blobs)
                b_a_h=a.h()
                if a.y()-black_jy+a.h()>black_jh*4/10 or abs(a.y()-black_jy)>=6 or lsx_b_blobs or a.x()<blue_x and hui_x<blue_x:b_a_h=0
                img.draw_rectangle(a.rect(),color=(225,225,225),thickness=2)
            if lx_b_blobs:
                b=find_max(lx_b_blobs)
                b_b_h=b.h()
                if b.y()-black_jy+b.h()>black_jh*4/10 or abs(b.y()-black_jy)>=6 or rsx_b_blobs or b.x()>hui_x and blue_x<hui_x:b_b_h=0
                img.draw_rectangle(b.rect(),color=(225,225,225),thickness=2)
            if b_a_h>6 or b_b_h>6:
                if b_a_h>=b_b_h:frist_2=2
                if b_a_h<=b_b_h:frist_2=1
        #判断-黄色-上部是否偏移
        if run==8:
            #print('-----------------------')
            if lsx_y_blobs:img.draw_rectangle(find_max(lsx_y_blobs).rect(),color=(225,225,225),thickness=2)
            if rsx_y_blobs:img.draw_rectangle(find_max(rsx_y_blobs).rect(),color=(225,225,225),thickness=2)
            if rx_y_blobs:
                a=find_max(rx_y_blobs)
                y_a_h=a.h()
                #限制大小、差值，下部分色块，蓝色旁边
                if a.y()-yellow_jy+a.h()>yellow_jh*4/10 or abs(a.y()-yellow_jy)>=6 or lsx_y_blobs or a.x()<blue_x and hui_x<blue_x and hui_x2>blue_x:y_a_h=0
                img.draw_rectangle(a.rect(),color=(225,225,225),thickness=2)
            if lx_y_blobs:
                b=find_max(lx_y_blobs)
                y_b_h=b.h()
                if b.y()-yellow_jy+b.h()>yellow_jh*4/10 or abs(b.y()-yellow_jy)>=6 or rsx_y_blobs or b.x()>hui_x and blue_x<hui_x and hui_x2>blue_x:y_b_h=0
                img.draw_rectangle(b.rect(),color=(225,225,225),thickness=2)
            if y_a_h>6 or y_b_h>6:
                if y_a_h>=y_b_h:frist_2=2
                if y_a_h<=y_b_h:frist_2=1
        #当前黑色上面离中心最近的色块编号
        if run==4:
            min_x=min(black_r_cx,black_b_cx,black_g_cx,black_h_cx)
            if min_x!=240:
                if min_x==black_r_cx:five_3=2
                if min_x==black_b_cx:five_3=0
                if min_x==black_g_cx:five_3=1
                if min_x==black_h_cx:five_3=3
            else:five_3=4
        #当前黄色上面离中心最近的色块编号
        if run==8:
            min_x=min(yellow_r_cx,yellow_b_cx,yellow_g_cx,yellow_h_cx)
            if min_x!=240:
                if min_x==yellow_r_cx:five_3=2
                if min_x==yellow_b_cx:five_3=0
                if min_x==yellow_g_cx:five_3=1
                if min_x==yellow_h_cx:five_3=3
            else:five_3=4
        #我需要找的色块的编号
        if need=='1':five_2=1
        if need=='2':five_2=0
        if need=='3':five_2=2
        if need=='4':five_2=3
        #发送参数计算
        find_dui=find_3*100#计算堆放区发送参数
        frist=9*100+frist_2*10+frist_3#第一个参数，最后一位用于纠偏
        five=9*100+five_2*10+five_3#第五个参数堆放区调整

        #发送数据---------------------------------------------------------------------
        uart.write("%d%d%d%d%d%d%d"%(frist,find_dui+100,red_cx+100,dui_cx+100,five,bi_zhuang_x+100,999))#yes=101视野内有色块,match=101匹配，可以投放
        print("%d%d%d%d%d%d%d"%(frist,find_dui+100,red_cx+100,dui_cx+100,five,bi_zhuang_x+100,999))
        find_dui=0;red_cx=0;find_3=0;frist_3=0;dui_cx=0;frist_2=0;five_3=4;bi_zhuang_x=0;
        black_g_cx=240;black_b_cx=240;black_r_cx=240;black_h_cx=240;b_a_h=0;b_b_h=0;y_a_h=0;y_b_h=0;hui_x2=0
        #time.sleep(0.1)



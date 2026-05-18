#include "image.hpp"
#include "rot.hpp"
#include "math.h"
#include "camera_send.hpp"
#include <cassert>
//-------------------------------------------------------------------------------------------------------------------
//  @brief      快速大津      CYH-NYY
//  @return     uint8
//  @since      v1.1
//  Sample usage:   OTSU_Threshold = otsuThreshold(image_use_dvp[0]);//大津法阈值
//-------------------------------------------------------------------------------------------------------------------
uint8 last_threshold=0;
uint8 otsuThreshold(uint8 *image)
{
    uint16 width=UVC_WIDTH;
    uint16 height=UVC_HEIGHT-4;
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};//每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height/4;   //总像素点
    uint8 threshold = 0;

    uint8* data = image;  //指向像素数据的指针


    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
          //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;   //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];  //总平均灰度
    }


    float maxVariance=0.0;  //最大类间方差
    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        // float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差
		float diff = avgValue / w0 - u;
		float variance = diff * diff * w0 / (1 - w0);
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8)i;
        }
    }

    if(threshold>40 && threshold<200)
        last_threshold = threshold;
    else
        threshold = last_threshold;

    return threshold;

}

// void adaptiveThreshold(uint8_t* img_data, uint8_t* output_data, int width, int height, int block, uint8_t clip_value)
// {
//   assert(block % 2 == 1); // block必须为奇数
//   int half_block = block / 2;
//   for(int y=half_block; y<height-half_block; y++){
//     for(int x=half_block; x<width-half_block; x++){
//       // 计算局部阈值
//       int thres = 0;
//       for(int dy=-half_block; dy<=half_block; dy++){
//         for(int dx=-half_block; dx<=half_block; dx++){
//           thres += img_data[(x+dx)+(y+dy)*width];
//         }
//       }
//       thres = thres / (block * block) - clip_value;
//       // 进行二值化
//       output_data[x+y*width] = img_data[x+y*width]>thres ? 255 : 0;
//     }
//   }
// }


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找最长白列     CYH-NYY
// 参数说明
// 返回参数
// 使用示例     get_highest();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

int lwline = 80;   // 最长白列所在的列号（x坐标）
int lw = 119;      // 最长白列“顶端”的行号（y坐标，越小越高）
extern uint8 yuzhi;// 二值化阈值（白/黑分界）
void get_highest(void)
{
    lwline=80;
    lw=119;

    for (int i=60;i<120;i+=3)          //从60列到120列从下往上找最长白列，隔三列扫一次减少代码运行时间
    {
        if(image_use[80][i]>=yuzhi)    //每一列的80行为黑就不扫
        {
            for (int j=85;j>=0;j--)
            {
                //找到白黑黑跳变，消除噪点影响
                if ((image_use[j][i]>=yuzhi&&image_use[j-1][i]<yuzhi&&image_use[j-2][i]<yuzhi)||j<=5)
                {
                    //贪心算法求得最长白列
                    if (j<lw)
                    {
                      lw=j;
                      lwline=i;
                    }
                    break;
                }
            }
        }
    }
}

//方向向量数组
int guize[8][2]={
        {0,-1},
        {1,-1},
        {1,0},
        {1,1},
        {0,1},
        {-1,1},
        {-1,0},
        {-1,-1}
                };
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     黑白判断函数，八邻域爬线所需     CYH-NYY
// 参数说明
// 返回参数      x,y点k方向是黑还是白
// 使用示例      cbh(x坐标,y坐标,k方向);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
//

int hig=18;                        //起始扫线行上移行数
uint8 cbh(int x,int y,int k)
{
    int i=0,j=0;
    i=x+guize[k][0];
    j=y+guize[k][1];

    if (j>=UVC_HEIGHT-hig) return 1;                                    //底部黑框
    else if (j<=1||i<=3||i>=UVC_WIDTH-4)return 2;          //扫弦停止框
    else if (image_use[j][i]>=yuzhi) return 0;                          //白点
    else return 1;                                                                      //黑点
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     八邻域爬线      CYH-NYY
// 参数说明
// 返回参数
// 使用示例     get_BLY(输出左边界数组，输出左边界点数，输出右边界数组，输出右边界点数，起始扫线行上移行数,左右补线标志位);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void get_BLY(float L_line[][2],int *Lnum,float R_line[][2],int *Rnum,int len ,uint8_t L_Bu , uint8_t R_Bu )
{
	uint8 L_amount = 100, R_amount = 100;  //左右边界搜点时最多允许的点
    hig=len;
    int L_C=0,R_C=0;

    //爬初始点----------------------------------------------
    if(L_Bu==0)//无需补线时
    {
        for (int16 i=(int16)lwline;i>0;i--)
        {
            if (cbh(i,UVC_HEIGHT-hig-1,6)==1&&cbh(i+1,UVC_HEIGHT-hig-1,6)==1&&cbh(i+2,UVC_HEIGHT-hig-1,6)==0)
            {
                L_line[0][0]=i+2;
                break;
            }
            else if (cbh(i+1,UVC_HEIGHT-hig-1,6)==2)
            {
                L_line[0][0]=i+2;
                break;
            }
        }
        L_line[0][1]=(float)UVC_HEIGHT-hig-1;
    }
    if(R_Bu==0) //无需补线时
    {
        for (int16 i=(int16)lwline;i<UVC_WIDTH;i++)
        {
            if (cbh(i,UVC_HEIGHT-hig-1,2)==1&&cbh(i-1,UVC_HEIGHT-hig-1,2)==1&&cbh(i-2,UVC_HEIGHT-hig-1,2)==0)
            {
                R_line[0][0]=i-2;
                break;
            }
            else if (cbh(i-1,UVC_HEIGHT-hig-1,2)==2)
            {
                R_line[0][0]=i-2;
                break;
            }
        }
        R_line[0][1]=(float)UVC_HEIGHT-hig-1;
    }

    L_C=1;
    R_C=1;

    int l_dirl=6,l_dirr=2;
    int dirl=6  ,dirr=2 ;
    if(L_Bu)
    {
        l_dirl=0;
        dirl=0;
    }
  
    if(R_Bu)
    {
        l_dirr=0;
        dirr=0;
    }
    //左边界------------------------------------------------------

    for (int i=1;i<L_amount;i++)
    {
        // L_C++;
        int cishu=0;

        if (cbh(L_line[i-1][0],L_line[i-1][1],0)==2||cbh(L_line[i-1][0],L_line[i-1][1],1)==2||
                cbh(L_line[i-1][0],L_line[i-1][1],2)==2||cbh(L_line[i-1][0],L_line[i-1][1],3)==2||
                cbh(L_line[i-1][0],L_line[i-1][1],4)==2||cbh(L_line[i-1][0],L_line[i-1][1],5)==2||
                cbh(L_line[i-1][0],L_line[i-1][1],6)==2||cbh(L_line[i-1][0],L_line[i-1][1],7)==2) break;


        for (int j=0;j<8;j++)
        {
            if (cbh(L_line[i-1][0],L_line[i-1][1],(j+l_dirl+4)%8)==1&&
                    cbh(L_line[i-1][0],L_line[i-1][1],(j+l_dirl+4+1)%8)==0&&
                    cbh(L_line[i-1][0],L_line[i-1][1],(j+l_dirl+4+2)%8)==0)
            {
                dirl=(j+dirl+4+1)%8;
                break;
            }
            else
            {
                cishu++;
            }

        }
        l_dirl=dirl;

        if (cishu==8)
        {
            break;
        }
        else
        {
            L_line[i][0]=L_line[i-1][0]+guize[dirl][0];
            L_line[i][1]=L_line[i-1][1]+guize[dirl][1];
             L_C++;
        }
    }

    //右边界------------------------------------------------------

    for (int i=1;i<R_amount;i++)
    {
        // R_C++;
        int cishu=0;

        if (cbh(R_line[i-1][0],R_line[i-1][1],0)==2||cbh(R_line[i-1][0],R_line[i-1][1],1)==2||
                cbh(R_line[i-1][0],R_line[i-1][1],2)==2||cbh(R_line[i-1][0],R_line[i-1][1],3)==2||
                cbh(R_line[i-1][0],R_line[i-1][1],4)==2||cbh(R_line[i-1][0],R_line[i-1][1],5)==2||
                cbh(R_line[i-1][0],R_line[i-1][1],6)==2||cbh(R_line[i-1][0],R_line[i-1][1],7)==2) break;

        for (int j=0;j<8;j++)
        {
            if (cbh(R_line[i-1][0],R_line[i-1][1],(8-j+l_dirr+4)%8)==1&&
                    cbh(R_line[i-1][0],R_line[i-1][1],(8-j+l_dirr+4-1)%8)==0&&
                    cbh(R_line[i-1][0],R_line[i-1][1],(8-j+l_dirr+4-2)%8)==0)
            {
                dirr=(8-j+l_dirr+4-1)%8;
                break;
            }
            else
            {
                cishu++;
            }
        }
        l_dirr=dirr;

        if (cishu==8)
        {
            break;
        }
        else
        {
            R_line[i][0]=R_line[i-1][0]+guize[dirr][0];
            R_line[i][1]=R_line[i-1][1]+guize[dirr][1];
             R_C++;
        }
    }

    *Lnum=L_C;
    *Rnum=R_C;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取中线     CYH-NYY
// 参数说明
// 返回参数
// 使用示例     get_mid(输入边线,输出边线,边线点数量);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
// float Lin_MID[50][2];     //左中线结构体
// float Lout_MID[50][2];
// float Rin_MID[50][2];     //左中线结构体
// float Rout_MID[50][2];
// int Lmidnum=0,Lmidnum2=0;
// int Rmidnum=0,Rmidnum2=0;

uint16 rood_wid=45;//赛道宽度

void get_mid_L(float line[][2],int num,float mid[][2],int *num2,int approx_num)
{
    int mnum=1;
    float dist=rood_wid/2;
    mid[0][0]=UVC_WIDTH/2;  //中线的第一个点固定在图像底部中心
    // mid[0][1]=UVC_HEIGHT-2; 
     mid[0][1] = line[0][1] + dist;        //UVC_HEIGHT-2
    for (int i = 0; i < num; i++)
    {
        float dx = line[clip(i + approx_num, 0, num - 1)][0] - line[clip(i - approx_num, 0, num - 1)][0];
        float dy = line[clip(i + approx_num, 0, num - 1)][1] - line[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        if (line[i][1] + dx * dist>UVC_HEIGHT-2) continue;
        else
        {
            mid[mnum][0] = line[i][0] - dy * dist;
            mid[mnum][1] = line[i][1] + dx * dist;
        }
        mnum++;
    }
    *num2=mnum;
}

void get_mid_R(float line[][2],int num,float mid[][2],int *num2,int approx_num)
{
    int mnum=1;
    float dist=rood_wid/2;      //中线的第一个点固定在图像底部中心
    mid[0][0]=UVC_WIDTH/2;
    // mid[0][1]=UVC_HEIGHT-2; 
    mid[0][1] = line[0][1] + dist;
    // for(int i =0;i<8;i++)
    // {
    //     if(line[i][0]<20||line[i][1]<20)
    //     {
    //         line[i][0]=UVC_WIDTH/2+16;
    //         line[i][1]=UVC_HEIGHT-2-i*2;
    //     }
    // }
    for (int i = 0; i < num; i++)
    {
        float dx = line[clip(i + approx_num, 0, num - 1)][0] - line[clip(i - approx_num, 0, num - 1)][0];
        float dy = line[clip(i + approx_num, 0, num - 1)][1] - line[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        if (line[i][1] - dx * dist>UVC_HEIGHT-2) continue;
        else
        {
            mid[mnum][0] = line[i][0] + dy * dist;
            mid[mnum][1] = line[i][1] - dx * dist;
        }
        mnum++;
    }
    *num2=mnum;
}



int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}


void touxian(float line[][2],int *num)
{
    for (int i=0;i<*num;i++)
    {
        int x=0,y=0;
        x=Xrot_point(line[i][0],line[i][1]);
        y=Yrot_point(line[i][0],line[i][1]);
        if (x>0&&x<UVC_WIDTH&&y>0&&y<UVC_HEIGHT)
        {
            line[i][0]=(float)x;
            line[i][1]=(float)y;
        }
        else
        {
            *num=i;
            break;
        }
    }
}

// 点集三角滤波   
void blur_points(float img_in[][2], float imag_out[][2], int num, int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++)
    {
        imag_out[i][0] = imag_out[i][1] = 0;
        for (int j = -half; j <= half; j++)
        {
            int fu=i+j;
            if (fu<0)fu=0;
            if (fu>num-1)fu=num-1;
            imag_out[i][0] += img_in[fu][0] * (half + 1 - abs(j));
            imag_out[i][1] += img_in[fu][1] * (half + 1 - abs(j));
        }
        imag_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        imag_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}

// void blur_points(float pts_in[][2],  float pts_out[][2], int num,int kernel)
// {
//     assert(kernel % 2 == 1);
//     int half = kernel / 2;
//     for (int i = 0; i < num; i++) 
//     {
//         pts_out[i][0] = pts_out[i][1] = 0;
//         for (int j = -half; j <= half; j++) 
//         {
//             pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
//             pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
//         }
//         pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
//         pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
//     }
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等距采样    
// 参数说明
// 返回参数
// 使用示例     resample_points2(输入点集,输入点集数量,输出点集,输出点集数量,点间距);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    if (num1 < 0)
    {
        *num2 = 0;
        return;
    }
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    int len = 1;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];

        do {
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;

            float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;

            if (r0 < 0 || r1 < 0) break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);

    }
    *num2 = len;
}

void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    float remain = 0.f;
    int len = 0;
    for(int i=0; i<num1-1 && len < *num2; i++){
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        float dn = sqrt(dx*dx+dy*dy);
        dx /= dn;
        dy /= dn;

        while(remain < dn && len < *num2){
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;
            
            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     求夹角 
// 参数说明
// 返回参数
// 使用示例     local_angle_points(输入点集,输入点集数量,输出角集，点间距);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 角度变化率非极大抑制  
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++) 
    {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) 
        {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

float conmax=0;
void guai_mum(float Edge[][2], int num, float ang_out[],int *posi,bool *if_zhi)
{
    conmax=0;
    *posi=0;
    *if_zhi=1;
    float ang_in[num+5]={0};
    local_angle_points(Edge,num,ang_in,5); 
                       
    nms_angle(ang_in,num,ang_out,10);
                       
        for (int i=0;i<num;i++)
        {
            if (ang_out[i]==0)continue;
            int im1 = clip(i - 5, 0, num - 1);
            int ip1 = clip(i + 5, 0, num - 1);
            float conf = fabs(ang_out[i]) - (fabs(ang_out[im1]) + fabs(ang_out[ip1])) / 2;
            if (conf>1.05&&conf<2)
            {
                *posi=i;
            }
            if (*posi>0)
            {
                break;
            }
        }
        
    /*最小二乘法判断直道*/
    *if_zhi=0;
 
    if (num>=25)
    {
        
        float x1=Edge[2][0];
        float y1=Edge[2][1];
        float x2=Edge[num-3][0];
        float y2=Edge[num-3][1];
        float A=1/(x1-x2);
        float B=1/(y2-y1);
        float C=y2/(y1-y2)-x2/(x1-x2);
        float D=0;
        for (int i=3;i<num-3;i++)
        {
            float x0=Edge[i][0];
            float y0=Edge[i][1];
            D+=fabs(A*x0+B*y0+C)*fabs(A*x0+B*y0+C)/(A*A+B*B)/(num-6);
        }
        conmax=D;

        if (D<=2.5)
        {
            *if_zhi=1;
        }
    }
}

//快速平方根
float SquareRootFloat(float number)
{
    int32_t i;
    float x, y;
    const float f = 1.5F;

    x = number * 0.5F;
    y = number;

    memcpy(&i, &y, sizeof(float));   // 安全转换
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(int32_t)); // 再转回来

    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));

    return number * y;
}

//把数组的一段点（从 location2 到 step-1）复制到 pts_out 数组，从 location1 开始写入
uint8 Arry_roll(float pts_in[][2], float pts_out[][2], uint8 location1, uint8 location2, uint8 step, uint8 step_Max)
{
    uint8 j = location1; // 输出数组的起始索引，从 location1 开始写入 pts_out

    // 从输入数组 pts_in 的 location2 索引开始，到 step 索引结束
    for(int16 i = location2; i < step; i++)
    {
        // 将输入数组当前点复制到输出数组
        pts_out[j][0] = pts_in[i][0];       // 复制 x 坐标
        pts_out[j++][1] = pts_in[i][1];     // 复制 y 坐标，并将 j 自增

        // 如果输出数组索引超过最大长度，停止复制
        if(j >= step_Max) break;
    }

    return j; // 返回输出数组最后写入的索引位置
}

//  location1[2]     // 贝塞尔曲线起点 {x, y}
//  location2[2]     // 贝塞尔曲线控制点 {x, y}，决定曲线弯曲程度
//  location3[2]     // 贝塞尔曲线终点 {x, y}
//  pts[][2]         // 存储生成的点的数组
//  add_location     // pts数组起始写入位置
//  dist             // 曲线上采样点的最小间距（像素距离）
uint8 add_three_point_bezier(uint8 location1[2], uint8 location2[2], uint8 location3[2],
                             float pts[][2], uint8 add_location, uint8 dist)
{
    if (add_location >= UVC_WIDTH) {
        return 0;
    }

    float dx1 = location2[0] - location1[0];
    float dy1 = location2[1] - location1[1];
    float len1 = SquareRootFloat(dx1 * dx1 + dy1 * dy1);

    float dx2 = location3[0] - location2[0];
    float dy2 = location3[1] - location2[1];
    float len2 = SquareRootFloat(dx2 * dx2 + dy2 * dy2);

    float curve_length = (len1 + len2) * 0.8f;

    uint8 num_points = curve_length / dist + 1;
    if (num_points > (UVC_WIDTH - add_location)) {
        num_points = UVC_WIDTH - add_location;
    }

    float step = 1.0f / (num_points - 1);
    float t = 0.0f;
    uint8 len = 0;
    float x, y, prev_x = 0, prev_y = 0;
    float current_dist = 0;

    pts[add_location + len][0] = location1[0];
    pts[add_location + len][1] = location1[1];
    prev_x = location1[0];
    prev_y = location1[1];
    len++;

    t = step;
    while (t <= 1.0f && len < (UVC_WIDTH - add_location)) {
        float t_inv = 1.0f - t;
        float t_inv_squared = t_inv * t_inv;
        float t_squared = t * t;

        x = t_inv_squared * location1[0] + 2 * t_inv * t * location2[0] + t_squared * location3[0];
        y = t_inv_squared * location1[1] + 2 * t_inv * t * location2[1] + t_squared * location3[1];

        float dx = x - prev_x;
        float dy = y - prev_y;
        current_dist = SquareRootFloat(dx * dx + dy * dy);

        if (current_dist >= dist) {
            if (x < 0 || x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT) break;

            pts[add_location + len][0] = (uint8)x;
            pts[add_location + len][1] = (uint8)y;
            len++;

            prev_x = x;
            prev_y = y;
            current_dist = 0;
        }

        t += step;
    }

    if (len < (UVC_WIDTH - add_location) &&
        !(location3[0] < 0 || location3[0] >= UVC_WIDTH || location3[1] < 0 || location3[1] >= UVC_HEIGHT)) {
        pts[add_location + len][0] = location3[0];
        pts[add_location + len][1] = location3[1];
        len++;
    }

    return len;
}

// uint8 Arry_Filter_2(uint8 pts_in[][2],uint8 step)
// {
//     uint8 min_y = Start_high,id = 0;
//     for(uint8 i = 0; i < step; i++)
//     {
//         if(pts_in[i][1] < min_y)
//         {
//             min_y = pts_in[i][1];
//             id = i;
//         }
//     }
//     return id;
// }
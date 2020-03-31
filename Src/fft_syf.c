// ConsoleApplication1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//#include <iostream>
#include <math.h>
#include <stdio.h> 
#include "fft_syf.h"


//using namespace std;



/*定义复数类型*/
//typedef struct {
//	double r;// real;
//	double i;// img;
//}complex;

extern double cc[size_x];	//输入序列实部
complex x[size_x]; /*输入序列,变换核*/  //变换核为离散数列,就是需要变换的波形 2020-3-5

double PI;         /*圆周率*/
void fft(double mode);     /*快速傅里叶变换*/
void initW(double mode);   /*初始化变换核*/
void swap(void); /*变址*/
void add(complex, complex, complex *); /*复数加法*/
void mul(complex, complex, complex *); /*复数乘法*/
void sub(complex, complex, complex *); /*复数减法*/
void output(void);/*输出快速傅里叶变换的结果*/

const double DFT = 2.0, IDFT = -2.0;///进行求值，第二个参数传DFT，插值传IDFT
int aaa;
double	mo[13];

//int main()
//{
//int i;                             /*输出结果*/

//	PI = atan(1) * 4;
//	printf(" 输出DIT方法实现的FFT结果\n");


//	for (i = 0; i < size_x; i++) //读取数列
//	{
//		x[i].r= cc[i];
//		x[i].i = 0;
//	/*	printf("请输入第%d个序列：", i);
//		scanf("%lf%lf", &x[i].r, &x[i].i);*/
//	}


//	
////	initW(IDFT);//调用变换核,计算旋转因子
//	fft(IDFT);//调用快速傅里叶变换  
//	printf("各阶谐波含量:");

//	for (i = 0; i < 13; i++)
//	{
//		mo[i]= sqrt(x[i].r*x[i].r+ x[i].i*x[i].i);
//		if (mo[i] > 0.0001)
//		{
//			printf("\r\n%d阶:", i);
//			printf("%.4f", x[i].r);
//			if (x[i].i >= 0)printf("+%.4fj", x[i].i);
//			else if (fabs(x[i].i) < 0.0001)printf("+%.4fj");
//			else printf("%.4fj", x[i].i);
//			printf(",\t", i);

//			printf("含量 %03.02f,\t", mo[i]);
//			//		函数声明:atan2(double y, double x);
//	//		用途：返回给定的 X 及 Y 坐标值的反正切值
//	//		fai = atan2(x[i].i, x[i].r)*180/PI;
//			//		fai = atan(x[i].i);
//			printf("偏移角 %.1f", atan2(x[i].i, x[i].r) * 180 / PI);
//			printf("\r\n");
//		}
//		
//	}

//}


/*快速傅里叶变换*/
void fft(double mode)
{
	unsigned short i = 0, j = 0, k = 0, m = 0,A0,A1;
	complex product;

//	printf("输入序列\n");
//	output(); 

	swap();  //调用变址函数  
//	printf("输出倒序后的序列\n");
//	output();


	for (i = 2; i <= size_x ; i<<=1)        /*一级蝶形运算 stage */
	{
		m = (i>>1);
		for (j = 0; j < size_x; j += i)     /*一组蝶形运算 group,每个group的蝶形因子乘数不同*/
		{
			A0 = j;
			A1 = j + m;
			for (k = 0; k < m; k++)        /*一个蝶形运算 每个group内的蝶形运算*/
			{
				mul(x[A1], W[size_x * k / i], &product);// W 的
				sub(x[A0], product, &x[A1]);
				add(x[A0], product, &x[A0]);
				A0++; A1++;
			}
		}
	}

	if (mode == IDFT)
	{
		x[0].r /= (size_x ); //第一个点就是直流分量，它的模值就是直流分量的N倍
		x[0].i /= (size_x );

		for (int i = 1; i < size_x; i++) //其他阶数模值就是该阶谐波分量的N / 2倍
		{
			x[i].r /= size_h;
			x[i].i /= size_h;
		}
	}
//	printf("输出FFT后的结果\n");
//	output();//调用输出傅里叶变换结果函数  	
			
}

//inline void FFT(Complex a[], double mode)



/*变址计算，将x(n)码位倒置*/
void swap(void)
{
	complex temp;
	unsigned short i, j, k, n, m;
//	double t;
//	t = (log(size_x) / log(2));
//	n = t;
//	if (t != (double)n) n++;//补齐2的n次方?
	n = 0;
	for (i = 2; i <= size_x; i <<= 1)
	{
		n++;
	}

//	if (n > 20) n = 20;

	for (i = 0; i < size_x; i++)
	{
		k = i; j = 0;
		
		for (m = n; m != 0; m--)//while ((t--) > 0)    //利用按位与以及循环实现码位颠倒  
		{
			j = j << 1;
			j |= (k & 1);
			k = k >> 1;
		}
		if (j > i)    //将x(n)的码位互换  
		{
			temp = x[i];
			x[i] = x[j];
			x[j] = temp;
		}
	}
//	output();
}



/*输出傅里叶变换的结果*/
void output(void)
{
	int i;
	printf("The result are as follows：\n");
	for (i = 0; i < size_x; i++)
	{
		printf("%.4f", x[i].r);
		if (x[i].i >= 0.0001)printf("+%.4fj\n", x[i].i);
		else if (fabs(x[i].i) < 0.0001)printf("\n");
		else printf("%.4fj\n", x[i].i);
	}
}


void add(complex a, complex b, complex *c)  //复数加法的定义
{
	c->r = a.r + b.r;
	c->i = a.i + b.i;
}


void mul(complex a, complex b, complex *c)  //复数乘法的定义  
{
	c->r = a.r*b.r - a.i*b.i;
	c->i = a.r*b.i + a.i*b.r;
}


void sub(complex a, complex b, complex *c)  //复数减法的定义  
{
	c->r = a.r - b.r;
	c->i = a.i - b.i;
}


#ifdef testW

/*快速傅里叶变换方法二??*/
void fft2(double mode)
{
	unsigned short i = 0, j = 0, k = 0, m = 0,A0,A1;
	complex product;

//	printf("输入序列\n");
//	output(); 

	swap();  //调用变址函数  
//	printf("输出倒序后的序列\n");
//	output();

	for (int i = 0; i < log(size_x) / log(2); i++)        /*一级蝶形运算 stage */
	{
		int h = 1 << i;
		for (int j = 0; j < size_x; j += 2 * h)     /*一组蝶形运算 group,每个group的蝶形因子乘数不同*/
		{
			for (int k = 0; k < h; k++)        /*一个蝶形运算 每个group内的蝶形运算*/
			{
				mul(x[j + k + h], W[size_x*k / 2 / h], &product);
				add(x[j + k], product, &up);
				sub(x[j + k], product, &down);
				x[j + k] = up;
				x[j + k + h] = down;
			}
		}
	}
	

	if (mode == IDFT)
	{
		x[0].r /= (size_x ); //第一个点就是直流分量，它的模值就是直流分量的N倍
		x[0].i /= (size_x );

		for (int i = 1; i < size_x; i++) //其他阶数模值就是该阶谐波分量的N / 2倍
		{
			x[i].r /= size_h;
			x[i].i /= size_h;
		}
	}
//	printf("输出FFT后的结果\n");
//	output();//调用输出傅里叶变换结果函数  	
			
}


/*初始化变换核，定义一个变换核，相当于旋转因子WAP*/
//旋转因子在阶数固定时为固定值 2020-3-5
void initW(double mode)
{
	int i;
	complex *pW; /*变换核*/

	pW = (complex *)malloc(sizeof(complex) * size_h);  //生成变换核,分配空间,旋转因子只需要阶数的一半

//	printf("旋转因子 %d \n", size_x);
	printf("\nconst complex W[%d] ={", size_h);
	
	for (i = 0; i < size_h; i++)
	{
		pW[i].r = cos(mode * PI / size_x * i);   //用欧拉公式计算旋转因子  
		pW[i].i = -1 * sin(mode * PI / size_x * i);

		if ((i % 8) == 0) printf("\n");
		printf("{%f,%f},", pW[i].r, pW[i].i);
	}
	printf("\n};\n");
}
#endif


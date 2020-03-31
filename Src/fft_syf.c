// ConsoleApplication1.cpp : ���ļ����� "main" ����������ִ�н��ڴ˴���ʼ��������
//

//#include <iostream>
#include <math.h>
#include <stdio.h> 
#include "fft_syf.h"


//using namespace std;



/*���帴������*/
//typedef struct {
//	double r;// real;
//	double i;// img;
//}complex;

extern double cc[size_x];	//��������ʵ��
complex x[size_x]; /*��������,�任��*/  //�任��Ϊ��ɢ����,������Ҫ�任�Ĳ��� 2020-3-5

double PI;         /*Բ����*/
void fft(double mode);     /*���ٸ���Ҷ�任*/
void initW(double mode);   /*��ʼ���任��*/
void swap(void); /*��ַ*/
void add(complex, complex, complex *); /*�����ӷ�*/
void mul(complex, complex, complex *); /*�����˷�*/
void sub(complex, complex, complex *); /*��������*/
void output(void);/*������ٸ���Ҷ�任�Ľ��*/

const double DFT = 2.0, IDFT = -2.0;///������ֵ���ڶ���������DFT����ֵ��IDFT
int aaa;
double	mo[13];

//int main()
//{
//int i;                             /*������*/

//	PI = atan(1) * 4;
//	printf(" ���DIT����ʵ�ֵ�FFT���\n");


//	for (i = 0; i < size_x; i++) //��ȡ����
//	{
//		x[i].r= cc[i];
//		x[i].i = 0;
//	/*	printf("�������%d�����У�", i);
//		scanf("%lf%lf", &x[i].r, &x[i].i);*/
//	}


//	
////	initW(IDFT);//���ñ任��,������ת����
//	fft(IDFT);//���ÿ��ٸ���Ҷ�任  
//	printf("����г������:");

//	for (i = 0; i < 13; i++)
//	{
//		mo[i]= sqrt(x[i].r*x[i].r+ x[i].i*x[i].i);
//		if (mo[i] > 0.0001)
//		{
//			printf("\r\n%d��:", i);
//			printf("%.4f", x[i].r);
//			if (x[i].i >= 0)printf("+%.4fj", x[i].i);
//			else if (fabs(x[i].i) < 0.0001)printf("+%.4fj");
//			else printf("%.4fj", x[i].i);
//			printf(",\t", i);

//			printf("���� %03.02f,\t", mo[i]);
//			//		��������:atan2(double y, double x);
//	//		��;�����ظ����� X �� Y ����ֵ�ķ�����ֵ
//	//		fai = atan2(x[i].i, x[i].r)*180/PI;
//			//		fai = atan(x[i].i);
//			printf("ƫ�ƽ� %.1f", atan2(x[i].i, x[i].r) * 180 / PI);
//			printf("\r\n");
//		}
//		
//	}

//}


/*���ٸ���Ҷ�任*/
void fft(double mode)
{
	unsigned short i = 0, j = 0, k = 0, m = 0,A0,A1;
	complex product;

//	printf("��������\n");
//	output(); 

	swap();  //���ñ�ַ����  
//	printf("�������������\n");
//	output();


	for (i = 2; i <= size_x ; i<<=1)        /*һ���������� stage */
	{
		m = (i>>1);
		for (j = 0; j < size_x; j += i)     /*һ��������� group,ÿ��group�ĵ������ӳ�����ͬ*/
		{
			A0 = j;
			A1 = j + m;
			for (k = 0; k < m; k++)        /*һ���������� ÿ��group�ڵĵ�������*/
			{
				mul(x[A1], W[size_x * k / i], &product);// W ��
				sub(x[A0], product, &x[A1]);
				add(x[A0], product, &x[A0]);
				A0++; A1++;
			}
		}
	}

	if (mode == IDFT)
	{
		x[0].r /= (size_x ); //��һ�������ֱ������������ģֵ����ֱ��������N��
		x[0].i /= (size_x );

		for (int i = 1; i < size_x; i++) //��������ģֵ���Ǹý�г��������N / 2��
		{
			x[i].r /= size_h;
			x[i].i /= size_h;
		}
	}
//	printf("���FFT��Ľ��\n");
//	output();//�����������Ҷ�任�������  	
			
}

//inline void FFT(Complex a[], double mode)



/*��ַ���㣬��x(n)��λ����*/
void swap(void)
{
	complex temp;
	unsigned short i, j, k, n, m;
//	double t;
//	t = (log(size_x) / log(2));
//	n = t;
//	if (t != (double)n) n++;//����2��n�η�?
	n = 0;
	for (i = 2; i <= size_x; i <<= 1)
	{
		n++;
	}

//	if (n > 20) n = 20;

	for (i = 0; i < size_x; i++)
	{
		k = i; j = 0;
		
		for (m = n; m != 0; m--)//while ((t--) > 0)    //���ð�λ���Լ�ѭ��ʵ����λ�ߵ�  
		{
			j = j << 1;
			j |= (k & 1);
			k = k >> 1;
		}
		if (j > i)    //��x(n)����λ����  
		{
			temp = x[i];
			x[i] = x[j];
			x[j] = temp;
		}
	}
//	output();
}



/*�������Ҷ�任�Ľ��*/
void output(void)
{
	int i;
	printf("The result are as follows��\n");
	for (i = 0; i < size_x; i++)
	{
		printf("%.4f", x[i].r);
		if (x[i].i >= 0.0001)printf("+%.4fj\n", x[i].i);
		else if (fabs(x[i].i) < 0.0001)printf("\n");
		else printf("%.4fj\n", x[i].i);
	}
}


void add(complex a, complex b, complex *c)  //�����ӷ��Ķ���
{
	c->r = a.r + b.r;
	c->i = a.i + b.i;
}


void mul(complex a, complex b, complex *c)  //�����˷��Ķ���  
{
	c->r = a.r*b.r - a.i*b.i;
	c->i = a.r*b.i + a.i*b.r;
}


void sub(complex a, complex b, complex *c)  //���������Ķ���  
{
	c->r = a.r - b.r;
	c->i = a.i - b.i;
}


#ifdef testW

/*���ٸ���Ҷ�任������??*/
void fft2(double mode)
{
	unsigned short i = 0, j = 0, k = 0, m = 0,A0,A1;
	complex product;

//	printf("��������\n");
//	output(); 

	swap();  //���ñ�ַ����  
//	printf("�������������\n");
//	output();

	for (int i = 0; i < log(size_x) / log(2); i++)        /*һ���������� stage */
	{
		int h = 1 << i;
		for (int j = 0; j < size_x; j += 2 * h)     /*һ��������� group,ÿ��group�ĵ������ӳ�����ͬ*/
		{
			for (int k = 0; k < h; k++)        /*һ���������� ÿ��group�ڵĵ�������*/
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
		x[0].r /= (size_x ); //��һ�������ֱ������������ģֵ����ֱ��������N��
		x[0].i /= (size_x );

		for (int i = 1; i < size_x; i++) //��������ģֵ���Ǹý�г��������N / 2��
		{
			x[i].r /= size_h;
			x[i].i /= size_h;
		}
	}
//	printf("���FFT��Ľ��\n");
//	output();//�����������Ҷ�任�������  	
			
}


/*��ʼ���任�ˣ�����һ���任�ˣ��൱����ת����WAP*/
//��ת�����ڽ����̶�ʱΪ�̶�ֵ 2020-3-5
void initW(double mode)
{
	int i;
	complex *pW; /*�任��*/

	pW = (complex *)malloc(sizeof(complex) * size_h);  //���ɱ任��,����ռ�,��ת����ֻ��Ҫ������һ��

//	printf("��ת���� %d \n", size_x);
	printf("\nconst complex W[%d] ={", size_h);
	
	for (i = 0; i < size_h; i++)
	{
		pW[i].r = cos(mode * PI / size_x * i);   //��ŷ����ʽ������ת����  
		pW[i].i = -1 * sin(mode * PI / size_x * i);

		if ((i % 8) == 0) printf("\n");
		printf("{%f,%f},", pW[i].r, pW[i].i);
	}
	printf("\n};\n");
}
#endif


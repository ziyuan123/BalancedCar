#include <Wire.h>
int a=1;
float Acc=0;

class MPU6050
{
  public:
    MPU6050();
    float Result[7];
    void OpenMpu();//开启mpu
    void ReadRes();//读取原始数据
    void Out();    //输出原始数据
    void Sum();  //数据处理
    void Set(int address,int sum); //倍率设置
};

MPU6050::MPU6050()
{   }

void MPU6050::OpenMpu()
{
  Serial.print("MPU6050  Open");  delay(1000);Serial.print(".");delay(1000);Serial.print(".");delay(1000);Serial.print(".\n");
  Wire.beginTransmission(0x68); //开启MPU6050的传输
  Wire.write(0x6B); //指定寄存器地址
  Wire.write(0); //写入一个字节的数据
  Wire.endTransmission(true); //结束传输，true表示释放总线
  Serial.println("Open Success!");
  delay(3000);
}

void MPU6050::ReadRes()
{
//  Serial.println("MPU6050  Reading...");
  Wire.beginTransmission(0x68); //开启MPU6050的传输
  Wire.write(0x3B); //指定寄存器地址
  Wire.requestFrom(0x68, 14, true); //将输据读出到缓存
  Wire.endTransmission(true); //关闭传输模式
  
  for(int i=0;i<7;i++)
  {
    Result[i] = Wire.read() << 8 | Wire.read(); //两个字节组成一个16位整数
    //Serial.print("the Result[");Serial.print(i);Serial.print("] have read as:    ");Serial.print(Result[i]);Serial.print("\n");
  }
  if(a){  Serial.println("Read Success!");a=0;}
}

void MPU6050::Out()
{
//  /*[0]*/ Serial.print("Result[0]: ");Serial.print(Result[0]);Serial.print("    ");//Acc  Y
//  /*[1]*/ Serial.print("Result[1]: ");Serial.print(Result[1]);Serial.print("    ");//X
//  /*[2]*/ Serial.print("Result[2]: ");Serial.print(Result[2]);Serial.print("    ");//Z
//  /*[3]*/ Serial.print("Result[3]: ");Serial.print(Result[3]);Serial.print("    ");//温度
//  /*[4]*/ Serial.print("Result[4]: ");Serial.print(Result[4]);Serial.print("    ");//Gyr   X
  /*[5]*/ Serial.print("Result[5]: ");Serial.print(Result[5]);Serial.print("\n");//Y
//  /*[6]*/ Serial.print("Result[6]: ");Serial.print(Result[6]);Serial.print("\n");//Z
}

void MPU6050::Sum()
{
  Result[0]=  2*9.8*Result[0]/32768;
  Result[5]=  -(250*Result[5]/32768);
}

void MPU6050::Set(int address,int sum)
{
  if(address==1) int a=0x1c;
  else int a=0x1b;
  Wire.beginTransmission(0x68); //开启MPU6050的传输
  Wire.write(a); //指定寄存器地址
  Wire.write(sum);
  Wire.endTransmission(true); //关闭传输模式
  Serial.print("Set the ");
  if(address==1){if(sum==0x00) Serial.print("Acc as ±2G"); if(sum==0x08) Serial.print("Acc as ±4G"); 
                 if(sum==0x010) Serial.print("Acc as ±8G"); if(sum==0x18) Serial.print("Acc as ±16G");}
  else if(address==2)        
                {if(sum==0x00) Serial.print("Gyr as ±250°/s"); if(sum==0x08) Serial.print("Gyr as ±500°/s"); 
                 if(sum==0x010) Serial.print("Gyr as ±1000°/s"); if(sum==0x18) Serial.print("Gyr as ±2000°/s");}
  delay(1000);Serial.print(".");delay(1000);Serial.print(".");delay(1000);Serial.print(".\n");
}

MPU6050 MyMpu;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  MyMpu.OpenMpu();
  MyMpu.Set(1,0x00);
  MyMpu.Set(2,0x00);
}

void loop() {
  // put your main code here, to run repeatedly:
  MyMpu.ReadRes();
  MyMpu.Sum();
//  MyMpu.Out();
  Acc=Kalman_Filter(MyMpu.Result[0],MyMpu.Result[5]);
  Serial.print(Acc);Serial.print("\n");
  
}




/******************************************************/

//卡尔曼滤波参数与函数
float dt=0.001;//注意：dt的取值为kalman滤波器采样时间
float angle, angle_dot;//角度和角速度
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
float R_angle=0.5 ,C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


//卡尔曼滤波
float Kalman_Filter(float angle_m, float gyro_m)//angleAx 和 gyroGy
{
        angle+=(gyro_m-q_bias) * dt;          //  经验/公式计算角度值
        angle_err = angle_m - angle;          //  测量误差
        Pdot[0]=Q_angle - P[0][1] - P[1][0];  //
        Pdot[1]=- P[1][1];
        Pdot[2]=- P[1][1];
        Pdot[3]=Q_gyro;
        P[0][0] += Pdot[0] * dt;
        P[0][1] += Pdot[1] * dt;
        P[1][0] += Pdot[2] * dt;
        P[1][1] += Pdot[3] * dt;
        PCt_0 = C_0 * P[0][0];
        PCt_1 = C_0 * P[1][0];
        E = R_angle + C_0 * PCt_0;
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
        t_0 = PCt_0;
        t_1 = C_0 * P[0][1];
        P[0][0] -= K_0 * t_0;
        P[0][1] -= K_0 * t_1;
        P[1][0] -= K_1 * t_0;
        P[1][1] -= K_1 * t_1;
        angle += K_0 * angle_err; //最优角度
        q_bias += K_1 * angle_err;
        angle_dot = gyro_m-q_bias;//最优角速度

        return angle;
}

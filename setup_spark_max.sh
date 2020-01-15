#download c++ api from rev website
http://www.revrobotics.com/content/sw/max/sdk/SPARK-MAX-SDK-v1.5.1.zip
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/rev
#cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/rev
#unzip -o /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-cpp/1.5.1/SparkMax-cpp-1.5.1-headers.zip
#unzip -o /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-driver/1.5.1/SparkMax-driver-1.5.1-headers.zip

mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/rev
#cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/rev

#unzip -o /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-cpp/1.5.1/SparkMax-cpp-1.5.1-linuxathena.zip
#unzip -o /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-cpp/1.5.1/SparkMax-cpp-1.5.1-linuxathenastatic.zip
#unzip -o /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-driver/1.5.1/SparkMax-driver-1.5.1-linuxathena.zip
#unzip -o /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-driver/1.5.1/SparkMax-driver-1.5.1-linuxathenastatic.zip


cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/rev
find /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-driver/1.5.1/ -name \*headers\*zip | xargs -n 1 unzip -o
find /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-cpp/1.5.1/ -name \*headers\*zip | xargs -n 1 unzip -o 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/rev 
find /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-driver/1.5.1/ -name \*linux\*zip | xargs -n 1 unzip -o
find /home/ubuntu/Downloads/maven/com/revrobotics/frc/SparkMax-cpp/1.5.1/ -name \*linux\*zip | xargs -n 1 unzip -o 


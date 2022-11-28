#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QSound>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    flag_blue(false),//初始化标志位为False
    flag_yellow(false),
    flag_green(false),
    flag_fan(false),
    maxTemp(50),
    maxHum(70),
    maxLight(50),
    flagTempWarn(true),
    flagHumWarn(true),
    flagLightWarn(true)
{
    ui->setupUi(this);
    initlcdNumber_system_time();//初始化lcd
    initPUshButton();//初始化按钮灯 和 风扇
    initUdpSocket();//调用socket
    initIpAndPort();
    initProgessBar();//调用初始化进度条和报警按钮
    initSclider();//初始化阈值相关函数
    initSoundTimer();//调用报警按钮和音效






}

MainWindow::~MainWindow()
{
    delete ui;

}
//初始化lcdnumber
void MainWindow::initlcdNumber_system_time()
{
    //设置lcd显示的位数 20位
    ui->lcdNumber_system_time->setDigitCount(20);
    //获取系统时间
    QString time=QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    //显示在LCD上
    ui->lcdNumber_system_time->display(time);
    //设置字体颜色
    ui->lcdNumber_system_time->setStyleSheet("color:rgb(125,150,60);");

    //创建定时器对象
    timer=new QTimer();
    timer->start(1000);

    connect(timer,SIGNAL(timeout()),this,SLOT(slotUpdatetime()));
    //在输入的IP地址和端口号中，不用每次写入端口IP
    ui->lineEdit_t_h_IP->setText("192.168.43.219");
    ui->lineEdit_t_h_Port->setText("20000");
    ui->lineEdit_ill_IP->setText("192.168.43.219");
    ui->lineEdit_ill_Port->setText("20000");
    ui->lineEdit_fan_IP->setText("192.168.43.219");
    ui->lineEdit_fan_Port->setText("20000");

}

void MainWindow::initPUshButton()
{


    ui->pushButton_blue_LED->setStyleSheet("color:blue; border-image: url(:/images/bulb_off.png);");
    ui->pushButton_yellow_LED->setStyleSheet("color:yellow;border-image: url(:/images/bulb_off.png);");
    ui->pushButton_green_LED->setStyleSheet("color:green;border-image: url(:/images/bulb_off.png);");
    ui->pushButton_fan->setStyleSheet("color:orange;border-image: url(:/images/fan_off.png);");


}
//初始化sockt
void MainWindow::initUdpSocket()
{
    //创建一个对象
    socket=new QUdpSocket;
    //绑定自己ip地址
    socket->bind(QHostAddress("192.168.43.180"),8080);
    //将信号与槽绑定
    connect(socket,SIGNAL(readyRead()),this,SLOT(slotRecv()));
}

void MainWindow::initIpAndPort()
{
    //设置IP和端口号提示文字的字体和颜色
    ui->label_ill_ip->setStyleSheet("color:yellow");
    ui->label_ill_Port->setStyleSheet("color:yellow");
    ui->label_t_h_ip->setStyleSheet("color:green");

    ui->label_t_h_Port->setStyleSheet("color:green");
    ui->label_fan_ip->setStyleSheet("color:blue");

    ui->label_fan_Port->setStyleSheet("color:blue");

}

void MainWindow::initProgessBar()
{
    ui->label_temperature->setStyleSheet("color:WHITE");
    ui->progressBar_t->setStyleSheet("QProgressBar{background:white;}"
                                       " QProgressBar::chunk{background:yellow}"
                                       " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

    ui->pushButton_t_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");


    ui->label_humidity->setStyleSheet("color:blue");
    ui->progressBar_h->setStyleSheet("QProgressBar{background:white;}"
                                       " QProgressBar::chunk{background:yellow}"
                                       " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

    ui->pushButton_h_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");


    ui->label_illumination->setStyleSheet("color:green");
    ui->progressBar_ill->setStyleSheet("QProgressBar{background:white;}"
                                       " QProgressBar::chunk{background:yellow}"
                                       " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

    ui->pushButton_ill_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");




}

void MainWindow::initSclider()
{
    //设置阈值字体的颜色

    QString str=QString("温度阈值:%1").arg(maxTemp);

    ui->label_temperature_set->setText(str);
    ui->label_temperature_set->setStyleSheet("color:blue");
    ui->horizontalSlider_t->setValue(maxTemp);

     str=QString("湿度阈值:%1").arg(maxHum);
    ui->label_humidity_set->setText(str);
    ui->label_humidity_set->setStyleSheet("color:yellow");
    ui->horizontalSlider_h->setValue(maxHum);


     str=QString("光照阈值:%1").arg(maxLight);
    ui->label_illumination_set->setText(str);
    ui->label_illumination_set->setStyleSheet("color:red");
    ui->horizontalSlider_ill->setValue(maxLight);

}

void MainWindow::initSoundTimer()
{
    //创建音效对象，并设置循环播放
    soundTempWarn=new QSound(":sound/tempwarn.wav",this);
    soundHumWarn=new QSound(":sound/humwarn.wav",this);
    soundLightWarn=new QSound(":sound/lightwarn.wav",this);
    soundTempWarn->setLoops(-1);
    soundHumWarn->setLoops(-1);
    soundLightWarn->setLoops(-1);
    //创建定时器对象
    timerTempWarn=new QTimer;
    timerHumWarn=new QTimer;
    timerLightWarn=new QTimer;
    //将信号绑定槽函数
    connect(timerTempWarn,SIGNAL(timeout()),this,SLOT(slotTempWarn()));
    connect(timerHumWarn,SIGNAL(timeout()),this,SLOT(slotHumWarn()));
    connect(timerLightWarn,SIGNAL(timeout()),this,SLOT(slotLightWarn()));

}

void MainWindow::on_pushButton_blue_LED_clicked()
{

    if(flag_blue)

    {

        ui->pushButton_blue_LED->setStyleSheet("color:blue;border-image: url(:/images/bulb_on.png);");
        //给STM32发送一个开LED命令
        //单片机的地址是192.168.43.180单片机绑定的端口号是20000
        socket->writeDatagram("LEDB1",QHostAddress(ui->lineEdit_ill_IP->text()),ui->lineEdit_ill_Port->text().toInt());
    }
    else
    {
        ui->pushButton_blue_LED->setStyleSheet("color:blue;border-image: url(:/images/bulb_off.png);");
        socket->writeDatagram("LEDB0",QHostAddress(ui->lineEdit_ill_IP->text()),ui->lineEdit_ill_Port->text().toInt());
    }


    flag_blue=!flag_blue;
}

void MainWindow::on_pushButton_yellow_LED_clicked()
{
    if(flag_yellow)
    {

        ui->pushButton_yellow_LED->setStyleSheet("color:yellow;border-image: url(:/images/bulb_on.png);");
        socket->writeDatagram("LEDY1",QHostAddress(ui->lineEdit_ill_IP->text()),ui->lineEdit_ill_Port->text().toInt());

    }
    else
    {

        ui->pushButton_yellow_LED->setStyleSheet("color:yellow;border-image: url(:/images/bulb_off.png);");
        socket->writeDatagram("LEDY0",QHostAddress(ui->lineEdit_ill_IP->text()),ui->lineEdit_ill_Port->text().toInt());
    }
    flag_yellow=!flag_yellow;
}

void MainWindow::on_pushButton_green_LED_clicked()
{
    if(flag_green)
    {

        ui->pushButton_green_LED->setStyleSheet("color:green;border-image: url(:/images/bulb_on.png);");
        socket->writeDatagram("LEDG1",QHostAddress(ui->lineEdit_ill_IP->text()),ui->lineEdit_ill_Port->text().toInt());
    }
    else
    {
        ui->pushButton_green_LED->setStyleSheet("color:green;border-image: url(:/images/bulb_off.png);");
        socket->writeDatagram("LEDG0",QHostAddress(ui->lineEdit_ill_IP->text()),ui->lineEdit_ill_Port->text().toInt());
    }
    flag_green=!flag_green;
}







void MainWindow::on_pushButton_fan_clicked()
{
    if(flag_fan)
    {

        ui->pushButton_fan->setStyleSheet("color:orange;border-image: url(:/images/fan_on.png);");
        socket->writeDatagram("FAN1",QHostAddress(ui->lineEdit_fan_IP->text()),ui->lineEdit_fan_Port->text().toInt());

    }
    else
    {
        ui->pushButton_fan->setStyleSheet("color:orange;border-image: url(:/images/fan_off.png);");
        socket->writeDatagram("FAN0",QHostAddress(ui->lineEdit_fan_IP->text()),ui->lineEdit_fan_Port->text().toInt());
    }
    flag_fan=!flag_fan;
}
//当定时器timer 每隔一秒发送一个timeout信号绑定的槽函数
void MainWindow::slotUpdatetime()
{
    //获取当前的系统时间，保存到time中
        QSound::play(":sound/time.wav");
        QString time=QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
        ui->lcdNumber_system_time->display(time);

}
//温度阈值发生变化
void MainWindow::on_horizontalSlider_t_valueChanged(int value)
{
    QSound::play(":sound/move.wav");
    maxTemp = value;
    QString str=QString("温度阈值:%1").arg(maxTemp);
    ui->label_temperature_set->setText(str);
}

void MainWindow::on_horizontalSlider_h_valueChanged(int value)
{
    QSound::play(":sound/move.wav");
    maxTemp = value;
    QString str=QString("湿度阈值:%1").arg(maxTemp);
    ui->label_humidity_set->setText(str);
}

void MainWindow::on_horizontalSlider_ill_valueChanged(int value)
{
    QSound::play(":sound/move.wav");
    maxTemp = value;
    QString str=QString("光照阈值:%1").arg(maxTemp);
    ui->label_illumination_set->setText(str);
}
//接收数据的槽函数
void MainWindow::slotRecv()
{
    QString s;
    QByteArray data;//用来保存接收到的数据
    while(socket->hasPendingDatagrams())//有未接收的数据
    {


        //重置data大小和数据报大小
        data.resize(socket->pendingDatagramSize());
        //读取数据据
        socket->readDatagram(data.data(),data.size());
        //将字节数组转换成字符串
        s= QVariant(data).toString();
        //调试，打印接收到的数据
        //"ht,35,67"或者"light,87"
        qDebug()<<s;
        //s "ht,35,67"将s字符串以，为分隔符，分隔为3个字符串
        //0 1 2
        //"ht" "35" "67"
//      qDebug()<<"--------";
        QStringList list = s.split(",");
        if(s.startsWith("ht"))//如果上传上来的数据以ht开头代表温湿度
        {
        qDebug()<<list[0]<<list[1]<<list[2];
        int temp =list[1].toDouble();//"35"--->35
        int hum =list[2].toDouble();//"67"--->67
        //将温度和湿度显示在进度条上面
        ui->progressBar_t->setValue(temp);
        ui->progressBar_h->setValue(hum);
        if(temp>maxTemp)
        {
            timerTempWarn->start(30);//启动定时器
            soundTempWarn->play();//播放报警音效

        }
        else
        {
            timerTempWarn->stop();
            soundTempWarn->stop();
            ui->progressBar_t->setStyleSheet("QProgressBar{background:white;}"
                                               " QProgressBar::chunk{background:yellow}"
                                               " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

            ui->pushButton_t_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");

        }
        if(hum>maxHum)

        {
            timerHumWarn->start(30);
            soundHumWarn->play();
        }
        else
        {
            timerHumWarn->stop();
            soundHumWarn->stop();
            ui->progressBar_h->setStyleSheet("QProgressBar{background:white;}"
                                               " QProgressBar::chunk{background:yellow}"
                                               " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

            ui->pushButton_h_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");

        }
        }
        else if(s.startsWith("light"))//以light为开头说明是光照强度
        {
            //“light，100”用spilt函数分割字符串之后变成"light" "100"  对应下标是0，1
            int light =list[1].toDouble();
            //将光照强度显示在光照进度条上
            ui->progressBar_ill->setValue(light);
            if(light>maxLight)
            {
                timerLightWarn->start(30);
                soundLightWarn->play();

            }
            else
            {
                timerLightWarn->stop();
                soundLightWarn->stop();
                //定时器暂停后
                ui->progressBar_ill->setStyleSheet("QProgressBar{background:white;}"
                                                   " QProgressBar::chunk{background:yellow}"
                                                   " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

                ui->pushButton_ill_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");
            }

        }


    }
}
//温度警告
void MainWindow::slotTempWarn()
{
    if(flagTempWarn)
    {
        ui->progressBar_t->setStyleSheet("QProgressBar{background:white;}"
                                           " QProgressBar::chunk{background:red}"
                                           " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

        ui->pushButton_t_alarm->setStyleSheet("border-image:url(:images/buttonsred.png)");
    }
    else
    {
        ui->progressBar_t->setStyleSheet("QProgressBar{background:white;}"
                                           " QProgressBar::chunk{background:yellow}"
                                           " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

        ui->pushButton_t_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");

    }
    //切换标志位
    flagTempWarn=!flagTempWarn;
}
//湿度警告
void MainWindow::slotHumWarn()
{
    if(flagTempWarn)
    {
        ui->progressBar_h->setStyleSheet("QProgressBar{background:white;}"
                                           " QProgressBar::chunk{background:red}"
                                           " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

        ui->pushButton_h_alarm->setStyleSheet("border-image:url(:images/buttonsred.png)");
    }
    else
    {
        ui->progressBar_h->setStyleSheet("QProgressBar{background:white;}"
                                           " QProgressBar::chunk{background:yellow}"
                                           " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

        ui->pushButton_h_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");

    }
    //切换标志位
    flagHumWarn=!flagHumWarn;
}
//光照强度警告
void MainWindow::slotLightWarn()
{
    if(flagTempWarn)
    {

        ui->progressBar_ill->setStyleSheet("QProgressBar{background:white;}"
                                           " QProgressBar::chunk{background:red}"
                                           " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

        ui->pushButton_ill_alarm->setStyleSheet("border-image:url(:images/buttonsred.png)");
    }
    else
    {

        ui->progressBar_ill->setStyleSheet("QProgressBar{background:white;}"
                                           " QProgressBar::chunk{background:yellow}"
                                           " QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");

        ui->pushButton_ill_alarm->setStyleSheet("border-image:url(:images/buttonsgreen.png)");

    }
    //切换标志位
    flagLightWarn=!flagLightWarn;
}



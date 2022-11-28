#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>//定时器
#include <QDateTime>//获取系统时间
#include <QDebug>//调试程序
#include <QSound>//播放音效
#include <QUdpSocket>//网络通信 UDP

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void initlcdNumber_system_time();//初始化LCDNumber
    void initPUshButton();//初始化灯风扇按钮
    void initUdpSocket();//初始化socket
    void initIpAndPort();//初始化IP和端口号
    void initProgessBar();//初始化进度条和报警按钮
    void initSclider();//初始化阈值块相关
    void initSoundTimer();//初始化音效对象和警告定时器

private slots:
    void slotUpdatetime();//更新显示时间

    void on_pushButton_blue_LED_clicked();

    void on_pushButton_yellow_LED_clicked();

    void on_pushButton_green_LED_clicked();

    void on_pushButton_fan_clicked();

    void on_horizontalSlider_t_valueChanged(int value);

    void on_horizontalSlider_h_valueChanged(int value);

    void on_horizontalSlider_ill_valueChanged(int value);
    void slotRecv();//接收数据的槽函数
    void slotTempWarn();

    void slotHumWarn();

    void slotLightWarn();

private:
    Ui::MainWindow *ui;
    QTimer* timer;//定义控制显示当前时间的定时器
    bool flag_blue;//定义一个标志位
    bool flag_yellow;
    bool flag_green;
    bool flag_fan;
    QUdpSocket* socket;//用于网络编程  UDP通信的套接字对象
    int maxTemp;//温度最大值
    int maxHum;//湿度
    int maxLight;//光照强度

    QSound* soundTempWarn;//温度
    QSound* soundHumWarn;//湿度
    QSound* soundLightWarn;//光照强度
    QTimer* timerTempWarn;//温度
    QTimer* timerHumWarn;//湿度
    QTimer* timerLightWarn;//光照强度
    //三个标志位
    bool flagTempWarn;
    bool flagHumWarn;
    bool flagLightWarn;

};

#endif // MAINWINDOW_H

# 概要
センサ番号取得するためにzenseを起動するだけのモジュール

# インストール
1. 依存パッケージをインストールする(userのオプションはあってもなくてもインストールできる状態であればOK)
```
pip install --user -r requirements.txt
```
2. 下のインストール説明に沿ってPicoZenseSDKをインストールし, 下のコマンドを入力する(docker-composeによるインストールの場合は基本この手順は不要)
https://github.com/teaminaho/aspara_robo/blob/master/cv_setup.md
```
sudo ./install.sh
```

3. zense_pywrapper_for_serialパッケージをインストールする(userオプションについては同様) 
```
python setup.py install --user
```

# 使用例
```
In [1]: from zense_pywrapper_for_serial import PyPicoZenseModuleForSerial

In [2]: zns = PyPicoZenseModuleForSerial(0)
Detected 2 devices.
sensor_idx_:0
SERIAL : PD71A1DGD6260045P

In [3]: print(zns.getSerialNumber())
PD71A1DGD6260045P

In [4]: delete zns
    delete zns

In [5]: del zns
```

# 使用例2(print_zense_serial.shを使用する場合)
1. print_zense_serial.shに実行権限を付与する
```
cd ./scripts
chmod +x print_zense_serial.sh
```
2. zenseを1台のみ作業PCに接続し, zenseのLEDが点灯し2, 3秒待つ
3. print_zense_serial.shを実行する
```
./print_zense_serial.sh
```
4. 結果を見る
```
inaho-04@inaho04-NUC7i7DNKE:~/tmp/zense_pywrapper_for_serial/scripts$ ./print_zense_serial.sh 
Detected 1 devices.
sensor_idx_:0
libva info: VA-API version 1.1.0
libva info: va_getDriverName() returns 0
libva info: Trying to open /usr/lib/x86_64-linux-gnu/dri/i965_drv_video.so
libva info: Found init function __vaDriverInit_1_1
libva info: va_openDriver() returns 0
SERIAL : PD71A1EGD8280137P
```

# 注意点
 - コード内でopencvは使用していないがsetup.pyのopencv参照の記述を削除するとエラーを吐く(センサAPIの方で陰的に参照している?)
 - 以前のコードではデストラクタ呼び出し時に必ずセグフォが発生していたがPsShutdown()の前にPsCloseDevice()を呼ぶことで解消された(ただ最新のコードではそもそもPsShutdown()を使っていない)


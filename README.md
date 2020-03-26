# Installation
1. Install PicoZenseSDK from https://www.picozense.com/en/sdk.html

2. Create /usr/lib/pkgconfig/libpicozense.pc:
```
prefix=/home/[*** put your PicoZenseSDK install location here ***]
exec_prefix=${prefix}
includedir=${prefix}/Include
libdir=${exec_prefix}/Lib/x64
Name: libpicozense
Description: The Library for Pico Zense
Version: 1.0.0
Cflags: -I${includedir}/
Libs: -L${libdir} -lpicozense_api
```

3. Write and add below settings to your ~/.bashrc or something.
```
export PICOZENSE_PATH="$HOME/Libraries/PicoZenseSDK"
export LD_LIBRARY_PATH="$PICOZENSE_PATH/Lib/x64:$LD_LIBRARY_PATH"
```

4. Install python dependency
```
pip install -r requirements.txt
```

5. Build cython code
```
python setup.py install
```

# Scripts
 - main.py : Generate TOML setting File
 - capture.py : Scripts to capture RGB-D images using Pico Zense

# Usage of capture.py
1. Connect your PicoZense and Wait until its Green LED turns on
2. Generate TOML setting file
```
python main.py
```
3. Run capture the script
```
python capture.py
```


# Installation
1. Install PicoZenseSDK from https://www.picozense.com/en/sdk.html

2. Install python dependency
```
pip install -r requirements.txt
```

3. Change below two lines in "setup.py" to your PicoZenseSDK Locations.
```
ZENSE_LIB_DIR = "/home/{}/Libraries/PicoZenseSDK/Lib/x64".format(os.environ.get('USER'))
ZENSE_INCLUDE_DIR = "/home/{}/Libraries/PicoZenseSDK/Include".format(os.environ.get('USER'))
```

3. Build cython code
```
python setup.py build_ext --inplace
```

# Scripts
 - main.py : Generate TOML setting File
 - capture.py : Scripts to capture RGB-D images using Pico Zense

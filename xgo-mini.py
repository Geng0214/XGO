# 本机为 XGO-MINI
from xgolib import XGO
dog = XGO("xgolite") 
version=dog.read_firmware()
if version[0]=='M':
    print('XGO-MINI')
    dog = XGO("xgomini")
    dog_type='M'
else:
    print('XGO-LITE')
    dog_type='L'
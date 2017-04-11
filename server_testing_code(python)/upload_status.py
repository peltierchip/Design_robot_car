#!/usr/bin/env python
#coding=utf8
 
import httplib, urllib
 
httpClient = None
try:
    params = urllib.urlencode({'start':1,'rain': 1, 'positionx': 1.025, 'positiony': 1.532})
    headers = {"Content-type": "application/x-www-form-urlencoded"
                    , "Accept": "text/plain"}
 
    httpClient = httplib.HTTPConnection("www.victomteng.net", 80, timeout=30)
    httpClient.request("POST", "/host.php", params, headers)
 
    response = httpClient.getresponse()
    print response.status
    print response.reason
    print response.read()
    print response.getheaders() #获取头信息
except Exception, e:
    print e
finally:
    if httpClient:
        httpClient.close()

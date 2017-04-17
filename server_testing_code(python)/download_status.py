#!/usr/bin/env python
#coding=utf8
 
import httplib
 
httpClient = None
 
try:
    httpClient = httplib.HTTPConnection('www.victomteng.net', 80, timeout=30)
    httpClient.request('GET', '/host.php')
 
    #response是HTTPResponse对象
    response = httpClient.getresponse()
    print response.status
    print response.reason
    print response.read()
except Exception, e:
    print e
finally:
    if httpClient:
        httpClient.close()

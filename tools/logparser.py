# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals
from collections import namedtuple
from struct import unpack, calcsize

def BinaryLogReader(fn):
    with open(fn, 'rb') as f:
        msg=[]
        lasttype=-1
        while True:
            try:
                type,byte=f.read(2)
            except ValueError:
                return
            if type!=lasttype:
                if msg: yield lasttype, str().join(msg)
                msg=[]
                lasttype=type
            msg.append(byte)

def parseheader(header):
    rtr=(ord(header[1])>>4)&0x1
    len=(ord(header[1]))&0x0f
    id=((ord(header[0]))<<3)|((ord(header[1]))>>5)
    return rtr, len, id

def Msgparser(fn):
    time=0.0
    timeperiod=0.2
    for type, msg in BinaryLogReader(fn):
        while msg:
            rtr,datalen,id = parseheader(msg)
            yield time, type, id, msg[2:2+datalen]
            if id==0x37F:#0x281:
                time+=timeperiod
            msg=msg[2+datalen:]

msgnt=namedtuple('msgnt',('name', 'parameters', 'unpackstr'))
class Msg:
    def __init__(self, name):
        self.name=name

msg_definition={
    0x281:msgnt(name="throtleinfo", parameters=('position',), unpackstr='<H'),
    0x201:msgnt(name="motorcontrol", parameters=('direction',), unpackstr='<B'),
    0x284:msgnt(name="encoderinfo", parameters=('left','right'), unpackstr='<HH'),
    0x182:msgnt(name="wheelangle", parameters=('angle',), unpackstr='<h'),
    }

def MsgInterpreter(fn):
    for time, type, id, data in Msgparser(fn):
        msg_origin={'\x01':'can', '\x00':'pc'}[type]
        if id in msg_definition:
            mdef=msg_definition[id]
            msg=Msg(mdef.name)
            for par,value in zip(mdef.parameters, unpack(mdef.unpackstr,data[:calcsize(mdef.unpackstr)])):
                msg.__dict__[par]=value
            yield time, msg_origin, msg

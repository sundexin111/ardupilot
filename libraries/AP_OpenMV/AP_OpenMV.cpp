/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
  OpenMV library
*/
#define AP_SERIALMANAGER_OPENMV_BAUD           115200
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX     64     //字节
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX     64

#include "AP_OpenMV.h"

extern const AP_HAL::HAL& hal;

//constructor
AP_OpenMV::AP_OpenMV(void)
{
    //默认对应串口为空
    _port = NULL;
    //解贞步骤为0
    _step = 0;
}

/*
 * init - perform required initialisation
 */
void AP_OpenMV::init(const AP_SerialManager &serial_manager)
{
    //找到openmv的串口，且不为空
    //SerialProtocol_OPEN_MV串口服务器
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))){
        //关闭流串口
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //初始化uart串口
        //打开串口并开始运行
        _port->begin(AP_SERIALMANAGER_OPENMV_BAUD,AP_SERIALMANAGER_OPENMV_BUFSIZE_RX,AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }
}

//定时调用
bool AP_OpenMV::update()
{
    if(_port == NULL)
        return false;
    //读取收到字节的数量
    int16_t numc = _port->available();
    uint8_t date;
    //校验和
    uint8_t checksum = 0;

    //从串口读数据
    for(int16_t i = 0;i<numc;i++)
    {
        date = _port->read();

        switch(_step){
        case 0:
            if(date == 0xA5)
                _step=1;
            break;

        case 1:
            if(date == 0xA5)
                _step=1;
            else
                _step=0;
            break;

        case 2:
            _cx_temp = date;
            _step = 3;
            break;

        case 3:
            _cy_temp = date;
            _step = 4;
            break;

        case 4:
            _step = 0;
            checksum = _cx_temp + _cy_temp;
            //解贞成功
            if(checksum == date){
                cx = _cx_temp;
                cy = _cy_temp;
                last_frame_ms = AP_HAL::millis;
                return true;
            }
            break;

        default:
            _step = 0;
        }
    }
    return false;
}

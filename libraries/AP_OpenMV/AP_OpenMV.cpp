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
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX     64     //�ֽ�
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX     64

#include "AP_OpenMV.h"

extern const AP_HAL::HAL& hal;

//constructor
AP_OpenMV::AP_OpenMV(void)
{
    //Ĭ�϶�Ӧ����Ϊ��
    _port = NULL;
    //���경��Ϊ0
    _step = 0;
}

/*
 * init - perform required initialisation
 */
void AP_OpenMV::init(const AP_SerialManager &serial_manager)
{
    //�ҵ�openmv�Ĵ��ڣ��Ҳ�Ϊ��
    //SerialProtocol_OPEN_MV���ڷ�����
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))){
        //�ر�������
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //��ʼ��uart����
        //�򿪴��ڲ���ʼ����
        _port->begin(AP_SERIALMANAGER_OPENMV_BAUD,AP_SERIALMANAGER_OPENMV_BUFSIZE_RX,AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }
}

//��ʱ����
bool AP_OpenMV::update()
{
    if(_port == NULL)
        return false;
    //��ȡ�յ��ֽڵ�����
    int16_t numc = _port->available();
    uint8_t date;
    //У���
    uint8_t checksum = 0;

    //�Ӵ��ڶ�����
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
            //����ɹ�
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

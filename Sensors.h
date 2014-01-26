// w/ white plug-ins 1 means dark, 0 means light. Reverse when white is plugged in.


#include "WPILib.h"
class IRSensor //The sensors looking at the floor lines
{
public:
	DigitalInput *Sensor;
	IRSensor::IRSensor(int channel)
	{
		Sensor=new DigitalInput(2, channel);
	}
	IRSensor::IRSensor(int channel, int slot)
	{
		Sensor=new DigitalInput(slot, channel);
	}
	bool IRSensor::onLine()
	{
		//if the sensor is on a line, it will return true.
		int num=Sensor->Get();
		if(num==!1)
			return true;
		else
			return false;
	}
	int IRSensor::Get()
	{
		return Sensor->Get();
	}
};
/*
int main()
{
	IRSensor IR1(1); //Left
	IRSensor IR2(2); //Center
	IRSensor IR3(3); //Right
	
	
	while(true)
	{
		cout << IR1.Get() << "     " << IR2.Get() << "     " << IR3.Get() << "     " << endl;
	}
	

	m_topRight->Set(1);
	m_topLeft->Set(1);
	m_bottomRight->Set(1);
	m_bottomLeft->Set(1);
	
	while(Cookies)
	{
		if(IR1->onLine()==true)
		{
			m_topRight->Set(1);
			m_topLeft->Set(0);
			m_bottomRight->Set(0);
			m_bottomLeft->Set(-1);
		}
		if(IR3->onLine()==true)
		{
			m_topRight->Set(0);
			m_topLeft->Set(1);
			m_bottomRight->Set(-1);
			m_bottomLeft->Set(0);
		}
		if(IR2->onLine()==true)
		{
			m_topRight->Set(1);
			m_topLeft->Set(1);
			m_bottomRight->Set(1);
			m_bottomLeft->Set(1);
		}
	}
}
*/

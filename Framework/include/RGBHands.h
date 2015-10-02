#ifndef RGBHANDS_H
#define RGBHANDS_H

#include "CM730.h"

#define RGBHandBroadcast 222
#define RGBHandID_R 101
#define RGBHandID_L 102

namespace Robot
{
	class RGBHands
	{
	public:
		enum REGISTERS
		{
			RED,
			GREEN,
			BLUE,
			ID,
			BAUD,
			MODE,
			FADETIME,
			ONTIME,
			OFFTIME,
			REPEATTIMES
		};

		enum MODES
		{
			MANUAL,
			FADE,
			FLASH
		};

	public:
		RGBHands( CM730 * p_cm730 )
		{
			m_CM730 = p_cm730;
		}

		~RGBHands(){}

		void SetRGB( int p_red, int p_green, int p_blue )
		{
			m_CM730->WriteByte(RGBHandBroadcast, MODE, MANUAL, 0);

			SetColors( p_red, p_green, p_blue );
		}

		void SetFlash( int p_red, int p_green, int p_blue, int p_ontime, int p_offtime, int p_repeattimes )
		{
			//TODO: I've been having trouble getting this mode to set. Calling twice as a workaround.
			m_CM730->WriteByte(RGBHandBroadcast, MODE, FLASH, 0);
			m_CM730->WriteByte(RGBHandBroadcast, MODE, FLASH, 0);
			
			SetColors( p_red, p_green, p_blue );

			m_CM730->WriteByte(RGBHandBroadcast, ONTIME, p_ontime, 0);
			m_CM730->WriteByte(RGBHandBroadcast, OFFTIME, p_offtime, 0);
			m_CM730->WriteByte(RGBHandBroadcast, REPEATTIMES, p_repeattimes, 0);
		}

	private:
		CM730 * m_CM730;

		void SetColors( int p_red, int p_green, int p_blue )
		{
			m_CM730->WriteByte(RGBHandBroadcast, RED, p_red, 0);
			m_CM730->WriteByte(RGBHandBroadcast, GREEN, p_green, 0);
			m_CM730->WriteByte(RGBHandBroadcast, BLUE, p_blue, 0);
		}
	};
}

#endif
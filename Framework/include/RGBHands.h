#ifndef RGBHANDS_H
#define RGBHANDS_H

#include "CM730.h"

#define ID_RGB_HAND_R 134 //0x86
#define ID_RGB_HAND_L 133 //0x85	

namespace Robot
{
	class RGBHands
	{
	public:
		enum REGISTERS
		{
			RED = 0x20,
			GREEN,
			BLUE,
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
			m_CM730->WriteByte(ID_RGB_HAND_R, MODE, MANUAL, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, MODE, MANUAL, 0);

			SetColors( p_red, p_green, p_blue );
		}

		void SetFlash( int p_red, int p_green, int p_blue, int p_ontime, int p_offtime, int p_repeattimes )
		{
			m_CM730->WriteByte(ID_RGB_HAND_R, MODE, FLASH, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, MODE, FLASH, 0);
			
			SetColors( p_red, p_green, p_blue );

			m_CM730->WriteByte(ID_RGB_HAND_R, ONTIME, p_ontime, 0);
			m_CM730->WriteByte(ID_RGB_HAND_R, OFFTIME, p_offtime, 0);
			m_CM730->WriteByte(ID_RGB_HAND_R, REPEATTIMES, p_repeattimes, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, ONTIME, p_ontime, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, OFFTIME, p_offtime, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, REPEATTIMES, p_repeattimes, 0);
		}

	private:
		CM730 * m_CM730;

		void SetColors( int p_red, int p_green, int p_blue )
		{
			m_CM730->WriteByte(ID_RGB_HAND_R, RED, p_red, 0);
			m_CM730->WriteByte(ID_RGB_HAND_R, GREEN, p_green, 0);
			m_CM730->WriteByte(ID_RGB_HAND_R, BLUE, p_blue, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, RED, p_red, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, GREEN, p_green, 0);
			m_CM730->WriteByte(ID_RGB_HAND_L, BLUE, p_blue, 0);
		}
	};
}

#endif
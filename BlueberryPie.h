/*******************************************************************************
BlueberryPie.h Class for controlling a BlueberryPie Device

Copyright(C) 2019  Howard James May

This file is part of the SweetMaker project

The SweetMaker SDK is free software: you can redistribute it and / or
modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The SweetMaker SDK is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

Contact me at sweet.maker@outlook.com
********************************************************************************
Release     Date                        Change Description
--------|-------------|--------------------------------------------------------|
1         24-Jun-2019   Initial release
*******************************************************************************/


#ifndef __BLUEBERRY_PIE_H__
#define __BLUEBERRY_PIE_H__

#include <StrawberryString.h>
#include "MidiBle.h"
//#include "BleMidi.h"

namespace SweetMaker
{

	/*
	* Enable this for Serial Debug
	*/
	//#define SWEET_MAKER_DEBUG

	class BlueberryPie : public StrawberryString
	{

		/*
		* These class methods provide the public interface to BlueberryPie which you
		* use in your program.
		*/
	public:
		/*
		* Use this to create and setup StrawberryString once at startup
		*/
		BlueberryPie();
		int init();
		MidiBle midiBle;


	protected:

	private:
		/*
		* These class methods are only for use by FizzyMint itself
		*/
//		BleMidi bleMidi;
	};
}

#endif



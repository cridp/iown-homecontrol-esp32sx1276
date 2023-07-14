#pragma once

#include <iohcRadio.h>


namespace IOHC
{
    class iohcReceiver : iohcRadio
    {
        private:
        static iohcRadio *iohcRadioPtr;
        iohcRadio()
        {}

        public:
        iohcRadio(const iohcRadio &obj) = delete;
        static iohcRadio *getInstance()
        {
            if (!iohcRadioPtr)
                iohcRadioPtr = new iohcRadio();
            return iohcRadioPtr;
        }
    };

    iohcRadio *iohcRadio::iohcRadioPtr = nullptr;
}
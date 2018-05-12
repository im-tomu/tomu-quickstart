// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file contains configuration/settings for the Gecko SDK-provided
// "capsense" driver at Gecko_SDK/kits/common/drivers/capsense.c.

#ifndef _CAPSENSECONFIG_H_
#define _CAPSENSECONFIG_H_

#define ACMP_CAPSENSE                       ACMP0
#define ACMP_CAPSENSE_CLKEN                 CMU_HFPERCLKEN0_ACMP0
#define PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE PRS_CH_CTRL_SOURCESEL_ACMP0
#define PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE PRS_CH_CTRL_SIGSEL_ACMP0OUT

#define ACMP_CHANNELS 8

#define BUTTON0_CHANNEL 0
#define BUTTON1_CHANNEL 1

#define CAPSENSE_CH_IN_USE { true, true, false, false, false, false, false, false }

#endif // _CAPSENSECONFIG_H_

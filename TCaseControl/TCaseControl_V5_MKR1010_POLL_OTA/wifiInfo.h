// Wifi network info.  Copy this file in the same folder as sketch for OTA/wifi credentials.
#define SECRET_SSID "OverlandFummins";
#define SECRET_PASS "Potato123!";

//MQTT info
#define MQTT_BROKER "192.168.6.10"



/* MQTT messaging structure

/Overland_Fummins
/Overland_Fummins/T-Case_Controller                    -- The module we are in
/Overland_Fummins/T-Case_Controller/Requested_State    -- What the dash switch says
/Overland_Fummins/T-Case_Controller/Current_State      -- Where the transfer case is
/Overland_Fummins/T-Case_Controller/Action             -- What are we currently doing
/Overland_Fummins/T-Case_Controller/NetworkStatus      -- Online?
/Overland_Fummins/T-Case_Controller/IP                 -- IP address
/Overland_Fummins/T-Case_Controller/SignalStrength     -- Wifi signal strength
/Overland_Fummins/T-Case_Controller/Info               -- Some info
/Overland_Fummins/T-Case_Controller/Warning            -- Non-critical stuff needing attention
/Overland_Fummins/T-Case_Controller/Alarm              -- Critical stuff






*/
#include <stdlib.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
//#include <ros/ros.h>

// sudo chown root:root ./exe
// sudo chmod a+s ./exe
#include "SimpleKalmanFilter.h"

SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.01);

/////////////////////////////////////add parameter////////////////////////////////////////////
char mac_addr_beacon[18]= {"DA:FA:03:97:C3:23"};	//first beacon mac addr
char mac_addr_beacon2[18]= {"D9:7B:87:DD:3C:78"};	//second beacon mac addr

/////////////////////////////////////add parameter////////////////////////////////////////////
int8_t rssi;
int8_t rssi2;	
double rssi_loc;
double rssi_loc2;   

int device , status;

struct hci_request ble_hci_request(uint16_t ocf, int clen, void * status, void * cparam){
	
	struct hci_request rq;
	memset(&rq, 0, sizeof(rq));
	rq.ogf = OGF_LE_CTL;
	rq.ocf = ocf;
	rq.cparam = cparam;
	rq.clen = LE_ADD_DEVICE_TO_WHITE_LIST_CP_SIZE;
	rq.rparam = status;
	rq.rlen = 1;
	return rq;
}
// Set BLE scan parameters.
void set_BLE_scan_parameters(int device ,int status){

	le_set_scan_parameters_cp scan_params_cp;
	memset(&scan_params_cp, 0, sizeof(scan_params_cp));
	scan_params_cp.type 			= 0x00; 
	scan_params_cp.interval 		= htobs(0x0010);
	scan_params_cp.window 			= htobs(0x0010);
	scan_params_cp.own_bdaddr_type 	= 0x00; // Public Device Address (default).
	scan_params_cp.filter 			= 0x00; // Accept all.

	struct hci_request scan_params_rq = ble_hci_request(OCF_LE_SET_SCAN_PARAMETERS, LE_SET_SCAN_PARAMETERS_CP_SIZE, &status, &scan_params_cp);

	if ( hci_send_req(device, &scan_params_rq, 1000) < 0 ) {
		hci_close_dev(device);
		perror("Failed to set scan parameters data.");
	}

}
// Set BLE events report mask.
void set_BLE_events_report_mask(int device ,int status){
	le_set_event_mask_cp event_mask_cp;
	memset(&event_mask_cp, 0, sizeof(le_set_event_mask_cp));
	int i = 0;
	for ( i = 0 ; i < 8 ; i++ ) event_mask_cp.mask[i] = 0xFF;

	struct hci_request set_mask_rq = ble_hci_request(OCF_LE_SET_EVENT_MASK, LE_SET_EVENT_MASK_CP_SIZE, &status, &event_mask_cp);

	if ( hci_send_req(device, &set_mask_rq, 1000) < 0 ) {
		hci_close_dev(device);
		perror("Failed to set event mask.");

	}

}
// Enable scanning.
le_set_scan_enable_cp scan_cp;
void enable_scanning(int device ,int status){

	//le_set_scan_enable_cp scan_cp;
	memset(&scan_cp, 0, sizeof(scan_cp));
	scan_cp.enable 		= 0x01;	// Enable flag.
	scan_cp.filter_dup 	= 0x00; // Filtering disabled.

	struct hci_request enable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &scan_cp);

	if ( hci_send_req(device, &enable_adv_rq, 1000) < 0 ) {
		hci_close_dev(device);
		perror("Failed to enable scan.");
	}
}
// Disable scanning.
void disable_scanning(int device ,int status){
	memset(&scan_cp, 0, sizeof(scan_cp));
	scan_cp.enable = 0x00;	// Disable flag.

	struct hci_request disable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &scan_cp);

	if ( hci_send_req(device, &disable_adv_rq, 1000) < 0 ) {
		hci_close_dev(device);
		perror("Failed to disable scan.");

	}

	hci_close_dev(device);
}
// Get Results.
void get_Results(int device ,int status){
	struct hci_filter nf;
	hci_filter_clear(&nf);
	hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
	hci_filter_set_event(EVT_LE_META_EVENT, &nf);

	if ( setsockopt(device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0 ) {
		hci_close_dev(device);
		perror("Could not set socket options\n");

	}
}

int main(int argc, char **argv){
	//ros::init(argc,argv,"beacon_scan");
	// /ROS_INFO("SCAN_START!!");
	
	// Get HCI device.
	device = hci_open_dev(hci_get_route(NULL));
	if ( device < 0 ) { 
		perror("Failed to open HCI device.");
		return 0; 
	}

	// Set BLE scan parameters.
	set_BLE_scan_parameters(device,status);

	// Set BLE events report mask.
    set_BLE_events_report_mask(device,status);
	
	// Enable scanning.
	enable_scanning(device,status);
	
	// Get Results.
	get_Results(device ,status);

	printf("Scanning....\n");

	uint8_t buf[HCI_MAX_EVENT_SIZE];
	evt_le_meta_event * meta_event;				    
	
	/////////////////////////////////////add parameter////////////////////////////////////////////
	le_advertising_info * info;
	le_advertising_info * info2;
	int len;
	int len2;
   
   while(1){
           
        len = read(device, buf, sizeof(buf));
		if ( len >= HCI_EVENT_HDR_SIZE ) {
			meta_event = (evt_le_meta_event*)(buf+HCI_EVENT_HDR_SIZE+1);
			if ( meta_event->subevent == EVT_LE_ADVERTISING_REPORT ) {
				uint8_t reports_count = meta_event->data[0];
				void * offset = meta_event->data + 1;

				while ( reports_count-- ) {
				    /////////////////////////////////////add parameter////////////////////////////////////////////
					info = (le_advertising_info *)offset;
					info2 = (le_advertising_info *)offset;

					char addr[18];
					char addr2[18];

					ba2str(&(info->bdaddr), addr);
					ba2str(&(info2->bdaddr), addr2);

					char a=0;
					char b=0;
				    /////////////////////////////////////mac_addr_beacon////////////////////////////////////////////
					for(int i=0 ; i<18 ;i++){
						if(addr[i] == mac_addr_beacon[i]) a++;
						if(i == 17){
							if(a==18) rssi = (int8_t)info->data[info->length];
						}

					}
				    /////////////////////////////////////mac_addr_beacon2////////////////////////////////////////////
					for(int i=0 ; i<18 ;i++){
						if(addr2[i] == mac_addr_beacon2[i]) b++;
						if(i == 17){
							if(b==18) rssi2 = (int8_t)info->data[info->length];
						}

					}
				    /////////////////////////////////////kalmanFilter////////////////////////////////////////////
                    rssi_loc = simpleKalmanFilter.updateEstimate((float)rssi);
					rssi_loc2 = simpleKalmanFilter.updateEstimate((float)rssi2); 
                    printf("RSSI_kalman1= %d\n",rssi);
                    printf("RSSI_kalman2= %d\n",rssi2);
                
                offset = info->data + info->length + 2;
				}
			}
		}
        
    }

	// Disable scanning.
    disable_scanning(device ,status);
    return 0;
}

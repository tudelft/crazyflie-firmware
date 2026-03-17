#ifndef __AODV_H__
#define __AODV_H__

//#define AODV_DEBUG_ENABLE

/* Queue Constants */
#define AODV_RX_QUEUE_SIZE 5
#define AODV_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)

/* AODV Message Constants */
#define AODV_ENABLE_HELLO
#define AODV_HELLO_INTERVAL 1000 // default 1 seconds
#define AODV_DESTINATION_ONLY 0
#define AODV_GRATUITOUS_REPLY 0
#define AODV_ROUTE_DISCOVERY_TIME 2000 // default 2 seconds
#define AODV_RERR_UNREACHABLE_DEST_SIZE_MAX ((ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 4) / sizeof(Unreachable_Dest_t))
#define AODV_RREQ_BUFFER_SIZE_MAX 15

typedef enum {
  AODV_RREQ = 1,
  AODV_RREP = 2,
  AODV_RERR = 3,
  AODV_RREP_ACK = 4,
} AODV_MESSAGE_TYPE;

/*
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |     Type      |J|R|G|D|U|   Reserved          |   Hop Count   |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                            RREQ ID                            |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                    Destination IP Address                     |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                  Destination Sequence Number                  |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                    Originator IP Address                      |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                  Originator Sequence Number                   |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct {
  AODV_MESSAGE_TYPE type: 8;
  struct {
    uint16_t J: 1; // Join flag
    uint16_t R: 1; // Repair flag
    uint16_t G: 1; // Gratuitous RREP flag
    uint16_t D: 1; // Destination only flag
    uint16_t U: 1; // Unknown sequence number flag
    uint16_t Reserved: 11;
  } flags;
  uint8_t hopCount;
  uint32_t requestId;
  UWB_Address_t destAddress;
  uint32_t destSeqNumber;
  UWB_Address_t origAddress;
  uint32_t origSeqNumber;
} __attribute__((packed)) AODV_RREQ_Message_t;

/*
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |     Type      |R|A|    Reserved     |Prefix Sz|   Hop Count   |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                     Destination IP address                    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                  Destination Sequence Number                  |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                    Originator IP address                      |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                           Lifetime                            |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct {
  AODV_MESSAGE_TYPE type: 8;
  struct {
    uint16_t R: 1; // Repair flag
    uint16_t A: 1; // Acknowledgment required
    uint16_t Reserved: 9;
    uint16_t prefixSize: 5;
  } flags;
  uint8_t hopCount;
  UWB_Address_t destAddress;
  uint32_t destSeqNumber;
  UWB_Address_t origAddress;
  Time_t lifetime;
} __attribute__((packed)) AODV_RREP_Message_t;

/*
    0                   1
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |     Type      |   Reserved    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct {
  AODV_MESSAGE_TYPE type: 8;
  uint8_t reserved;
} __attribute__((packed)) AODV_RREP_ACK_Message_t;

typedef struct {
  UWB_Address_t destAddress;
  uint32_t destSeqNumber;
} __attribute__((packed)) Unreachable_Dest_t;

/*
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |     Type      |N|          Reserved           |   DestCount   |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |            Unreachable Destination IP Address (1)             |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |         Unreachable Destination Sequence Number (1)           |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
   |  Additional Unreachable Destination IP Addresses (if needed)  |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |Additional Unreachable Destination Sequence Numbers (if needed)|
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct {
  AODV_MESSAGE_TYPE type: 8;
  struct {
    uint16_t N: 1; // No delete flag
    uint16_t Reserved: 15;
  } flags;
  uint8_t destCount; // The number of unreachable destinations included in the message
  Unreachable_Dest_t unreachableList[AODV_RERR_UNREACHABLE_DEST_SIZE_MAX];
} __attribute__((packed)) AODV_RERR_Message_t;

void aodvInit();
void aodvDiscoveryRoute(UWB_Address_t destAddress);
void aodvSendRERR(Unreachable_Dest_t *unreachableList, int count);

#endif

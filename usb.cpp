#define UDMX_CMD_SET_SINGLE 1
#define UDMX_CMD_SET_MULTI 2

#include "Arduino.h"
#include "dmx-dac.h"
#include <avr/cpufunc.h>

typedef uint8_t u8;
typedef uint16_t u16;

#include "USBCore.h"

#define UDMX_VID 0x16c0
#define UDMX_PID 0x05dc

#define EP_TYPE_CONTROL				0x00
#define EP_TYPE_BULK_IN				0x81
#define EP_TYPE_BULK_OUT			0x80
#define EP_TYPE_INTERRUPT_IN		0xC1
#define EP_TYPE_INTERRUPT_OUT		0xC0
#define EP_TYPE_ISOCHRONOUS_IN		0x41
#define EP_TYPE_ISOCHRONOUS_OUT		0x40

//==================================================================
//==================================================================

#define IMANUFACTURER 1
#define IPRODUCT 2
#define ISERIAL 3

static const u16 STRING_LANGUAGE[2] PROGMEM = {
	(3<<8) | (2+2),
	0x0409	// English
};

static const u16 STRING_IMANUFACTURER[13] PROGMEM = {
	(3<<8) | (2+2*12),
	'w','w','w','.','a','n','y','m','a','.','c','h'
};

static const u16 STRING_IPRODUCT[5] PROGMEM = {
	(3<<8) | (2+2*4),
	'u','D','M','X'
};

static const u16 STRING_ISERIAL[15] PROGMEM = {
	(3<<8) | (2+2*14),
	'p','b','r','o','o','k','2','0','1','4','0','3','1','6'
};

/* FIXME: Serial number? */
//	DEVICE DESCRIPTOR
static const DeviceDescriptor USB_DeviceDescriptor PROGMEM =
	D_DEVICE(0x00,0x00,0x00,64,UDMX_VID,UDMX_PID,0x100,IMANUFACTURER,IPRODUCT,ISERIAL,1);

//==================================================================
//==================================================================

/* FIXME: ??? */
volatile u8 _usbConfiguration = 0;

static inline void WaitIN(void)
{
	while (!(UEINTX & (1<<TXINI)));
}

static inline void ClearIN(void)
{
	UEINTX = ~(1<<TXINI);
}

static inline void WaitOUT(void)
{
	while (!(UEINTX & (1<<RXOUTI)))
		;
}

static inline u8 WaitForINOrOUT()
{
	while (!(UEINTX & ((1<<TXINI)|(1<<RXOUTI))))
		;
	return (UEINTX & (1<<RXOUTI)) == 0;
}

static inline void ClearOUT(void)
{
	UEINTX = ~(1<<RXOUTI);
}

void Recv(volatile u8* data, u8 count)
{
	while (count--)
		*data++ = UEDATX;
}

static inline u8 Recv8()
{
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
}

static inline void SetEP(u8 ep)
{
	UENUM = ep;
}

static inline u8 FifoByteCount()
{
	return UEBCLX;
}

static inline u8 ReceivedSetupInt()
{
	return UEINTX & (1<<RXSTPI);
}

static inline void ClearSetupInt()
{
	UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
}

static inline void Stall()
{
	UECONX = (1<<STALLRQ) | (1<<EPEN);
}

static inline u8 ReadWriteAllowed()
{
	return UEINTX & (1<<RWAL);
}

static inline u8 Stalled()
{
	return UEINTX & (1<<STALLEDI);
}

static inline u8 FifoFree()
{
	return UEINTX & (1<<FIFOCON);
}

static inline void ReleaseRX()
{
	UEINTX = 0x6B;	// FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static inline void ReleaseTX()
{
	UEINTX = 0x3A;	// FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
}

static inline u8 FrameNumber()
{
	return UDFNUML;
}

//==================================================================
//==================================================================

u8 USBGetConfiguration(void)
{
	return _usbConfiguration;
}

#define USB_RECV_TIMEOUT
class LockEP
{
	u8 _sreg;
public:
	LockEP(u8 ep) : _sreg(SREG)
	{
		cli();
		SetEP(ep & 7);
	}
	~LockEP()
	{
		SREG = _sreg;
	}
};

//	Number of bytes, assumes a rx endpoint
u8 USB_Available(u8 ep)
{
	LockEP lock(ep);
	return FifoByteCount();
}

//	Non Blocking receive
//	Return number of bytes read
int USB_Recv(u8 ep, void* d, int len)
{
	if (!_usbConfiguration || len < 0)
		return -1;
	
	LockEP lock(ep);
	u8 n = FifoByteCount();
	len = min(n,len);
	n = len;
	u8* dst = (u8*)d;
	while (n--)
		*dst++ = Recv8();
	if (len && !FifoByteCount())	// release empty buffer
		ReleaseRX();
	
	return len;
}

//	Recv 1 byte if ready
int USB_Recv(u8 ep)
{
	u8 c;
	if (USB_Recv(ep,&c,1) != 1)
		return -1;
	return c;
}

//	Space in send EP
u8 USB_SendSpace(u8 ep)
{
	LockEP lock(ep);
	if (!ReadWriteAllowed())
		return 0;
	return 64 - FifoByteCount();
}

//	Blocking Send of data to an endpoint
int USB_Send(u8 ep, const void* d, int len)
{
	if (!_usbConfiguration)
		return -1;

	int r = len;
	const u8* data = (const u8*)d;
	u8 timeout = 250;		// 250ms timeout on send? TODO
	while (len)
	{
		u8 n = USB_SendSpace(ep);
		if (n == 0)
		{
			if (!(--timeout))
				return -1;
			delay(1);
			continue;
		}

		if (n > len)
			n = len;
		len -= n;
		{
			LockEP lock(ep);
			if (ep & TRANSFER_ZERO)
			{
				while (n--)
					Send8(0);
			}
			else if (ep & TRANSFER_PGM)
			{
				while (n--)
					Send8(pgm_read_byte(data++));
			}
			else
			{
				while (n--)
					Send8(*data++);
			}
			if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE)))	// Release full buffer
				ReleaseTX();
		}
	}
	return r;
}

extern const u8 _initEndpoints[] PROGMEM;
const u8 _initEndpoints[] = 
{
	0,
};

#define EP_SINGLE_64 0x32	// EP0
#define EP_DOUBLE_64 0x36	// Other endpoints

static
void InitEP(u8 index, u8 type, u8 size)
{
	UENUM = index;
	UECONX = 1;
	UECFG0X = type;
	UECFG1X = size;
}

static
void InitEndpoints()
{
	UERST = 0x7E;	// And reset them
	UERST = 0;
}

static bool
set_multiple(int channel, int len)
{
  uint8_t n;
  uint8_t val;

  if (channel + len > 512)
    return false;
//  if (channel == 0)

  /* TODO: Implement timeout and error checking.  */
  while (len) {
      if (UEINTX & (1<<RXOUTI)) {
	  n = FifoByteCount();
	  while (n) {
	      if (len == 0) {
		  return false;
	      }
	      len--;
	      n--;
	      val = Recv8();
	      dmx_set_channel(channel, val);
	      channel++;
	  }
	  ClearOUT();
      } else {
#if 0
	  /* Give ISRs chance to run.  */
	  sei();
	  _NOP();
	  cli();
#endif
      }
  }
  return true;
}

//	Handle vendor device requests
static bool
VendorRequest(Setup& setup)
{
	int channel;
	uint16_t n;

	channel = setup.wIndex;
	if (channel < 0 || channel > 511)
	  return false;
	switch (setup.bRequest) {
	case UDMX_CMD_SET_SINGLE:
	    if (setup.wValueH)
	      return false;
	    dmx_set_channel(channel, setup.wValueL);
	    return true;
	case UDMX_CMD_SET_MULTI:
	    n = ((uint16_t)setup.wValueH << 8) | setup.wValueL;
	    if (setup.wLength < setup.wIndex || n > 512)
	      return false;
	    return set_multiple(channel, n);
	/* TODO: Implement bootloader switching.  */
	}
	return false;
}

int _cmark;
int _cend;
void InitControl(int end)
{
	SetEP(0);
	_cmark = 0;
	_cend = end;
}

static
bool SendControl(u8 d)
{
	if (_cmark < _cend)
	{
		if (!WaitForINOrOUT())
			return false;
		Send8(d);
		if (!((_cmark + 1) & 0x3F))
			ClearIN();	// Fifo is full, release this packet
	}
	_cmark++;
	return true;
};

//	Clipped by _cmark/_cend
int USB_SendControl(u8 flags, const void* d, int len)
{
	int sent = len;
	const u8* data = (const u8*)d;
	bool pgm = flags & TRANSFER_PGM;
	while (len--)
	{
		u8 c = pgm ? pgm_read_byte(data++) : *data++;
		if (!SendControl(c))
			return -1;
	}
	return sent;
}

//	Does not timeout or cross fifo boundaries
//	Will only work for transfers <= 64 bytes
//	TODO
int USB_RecvControl(void* d, int len)
{
	WaitOUT();
	Recv((u8*)d,len);
	ClearOUT();
	return len;
}

static const struct {
    ConfigDescriptor config;
    InterfaceDescriptor interface;
} AllConfigDescriptors PROGMEM = {
  D_CONFIG(sizeof(ConfigDescriptor) + sizeof(InterfaceDescriptor), 1),
  D_INTERFACE(0,0,0,0,0)
};

//	Construct a dynamic configuration descriptor
//	This really needs dynamic endpoint allocation etc
//	TODO
static
bool SendConfiguration(int maxlen)
{
	InitControl(maxlen);
	USB_SendControl(TRANSFER_PGM,&AllConfigDescriptors,sizeof(AllConfigDescriptors));
	return true;
}

static
bool SendDescriptor(Setup& setup)
{
	u8 t = setup.wValueH;
	if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
		return SendConfiguration(setup.wLength);

	InitControl(setup.wLength);

	u8 desc_length = 0;
	const u8* desc_addr = 0;
	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
	{
		desc_addr = (const u8*)&USB_DeviceDescriptor;
	}
	else if (USB_STRING_DESCRIPTOR_TYPE == t)
	{
		if (setup.wValueL == 0)
			desc_addr = (const u8*)&STRING_LANGUAGE;
		else if (setup.wValueL == IPRODUCT) 
			desc_addr = (const u8*)&STRING_IPRODUCT;
		else if (setup.wValueL == IMANUFACTURER)
			desc_addr = (const u8*)&STRING_IMANUFACTURER;
		else if (setup.wValueL == ISERIAL)
			desc_addr = (const u8*)&STRING_ISERIAL;
		else
			return false;
	}

	if (desc_addr == 0)
		return false;
	if (desc_length == 0)
		desc_length = pgm_read_byte(desc_addr);

	USB_SendControl(TRANSFER_PGM,desc_addr,desc_length);
	return true;
}

//	Endpoint 0 interrupt
ISR(USB_COM_vect)
{
    SetEP(0);
	if (!ReceivedSetupInt())
		return;

	Setup setup;
	Recv((u8*)&setup,8);
	ClearSetupInt();

	u8 requestType = setup.bmRequestType;
	if (requestType & REQUEST_DEVICETOHOST)
		WaitIN();
	else
		ClearIN();

    bool ok = true;
	if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
	{
		//	Standard Requests
		u8 r = setup.bRequest;
		if (GET_STATUS == r)
		{
			Send8(0);		// TODO
			Send8(0);
		}
		else if (CLEAR_FEATURE == r)
		{
		}
		else if (SET_FEATURE == r)
		{
		}
		else if (SET_ADDRESS == r)
		{
			WaitIN();
			UDADDR = setup.wValueL | (1<<ADDEN);
		}
		else if (GET_DESCRIPTOR == r)
		{
			ok = SendDescriptor(setup);
		}
		else if (SET_DESCRIPTOR == r)
		{
			ok = false;
		}
		else if (GET_CONFIGURATION == r)
		{
			Send8(1);
		}
		else if (SET_CONFIGURATION == r)
		{
			if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
			{
				InitEndpoints();
				_usbConfiguration = setup.wValueL;
			} else
				ok = false;
		}
		else if (GET_INTERFACE == r)
		{
		}
		else if (SET_INTERFACE == r)
		{
		}
	}
	else if (requestType == (REQUEST_HOSTTODEVICE + REQUEST_VENDOR + REQUEST_DEVICE))
	{
		InitControl(setup.wLength);		//	Max length of transfer
		ok = VendorRequest(setup);
	}

	if (ok)
		ClearIN();
	else
	{
		Stall();
	}
}

//	General interrupt
ISR(USB_GEN_vect)
{
	u8 udint = UDINT;
	UDINT = 0;

	//	End of Reset
	if (udint & (1<<EORSTI))
	{
		InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);	// init ep0
		_usbConfiguration = 0;			// not configured yet
		UEIENX = 1 << RXSTPE;			// Enable interrupts for ep0
	}

	//	Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
	if (udint & (1<<SOFI))
	{
	  /* TODO */
	}
}

void
usb_init(void)
{
	uint8_t flags;

	_usbConfiguration = 0;
#ifdef UHWCON
	UHWCON = 0x01;						// power internal reg
#endif
	USBCON = (1<<USBE)|(1<<FRZCLK);		// clock frozen, usb enabled
#if F_CPU == 16000000L
#ifdef PINDIV
	flags = (1<<PINDIV);
#else
	flags = (1<<PLLP0);
#endif
#elif F_CPU == 8000000L
	flags = 0;
#else
#error USB requires either 8 or 16 MHz xtal
#endif
	PLLCSR = flags;
	PLLCSR = flags|(1<<PLLE);
	while (!(PLLCSR & (1<<PLOCK)))		// wait for lock pll
		;

	// Some tests on specific versions of macosx (10.7.3), reported some
	// strange behaviuors when the board is reset using the serial
	// port touch at 1200 bps. This delay fixes this behaviour.
	delay(1);

	flags = (1<<USBE);
#ifdef OTGPADE
	flags |= (1<<OTGPADE);
#endif
	USBCON = flags;						// start USB clock
	UDIEN = (1<<EORSTE)|(1<<SOFE);		// Enable interrupts for EOR (End of Reset) and SOF (start of frame)
	UDCON = 0;							// enable attach resistor
}

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Peter Lawrence
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/*
  Theory of operation:

  The M463 USBD peripheral has 12 "EP"s, but each is simplex,
  so two collectively (peripheral nomenclature of "EP0" and "EP1") are needed to
  implement USB EP0.  PERIPH_EP0 and PERIPH_EP1 are used by this driver for
  EP0_IN and EP0_OUT respectively.  This leaves up to six for user usage.
*/

#include "tusb_option.h"

#if CFG_TUD_ENABLED &&  (CFG_TUSB_MCU == OPT_MCU_M463) 

#include "device/dcd.h"
#include "NuMicro.h"

// Since TinyUSB doesn't use SOF for now, and this interrupt too often (1ms interval)
// We disable SOF for now until needed later on
#ifndef USE_SOF
#  define USE_SOF     0
#endif

/* allocation of USBD RAM for Setup, EP0_IN, and and EP_OUT */
//#define PERIPH_SETUP_BUF_BASE  0
//#define PERIPH_SETUP_BUF_LEN   64
#define PERIPH_EP0_BUF_BASE    0
#define PERIPH_EP0_BUF_LEN     CFG_TUD_ENDPOINT0_SIZE
#define PERIPH_EP1_BUF_BASE    (PERIPH_EP0_BUF_BASE + PERIPH_EP0_BUF_LEN)
#define PERIPH_EP1_BUF_LEN     CFG_TUD_ENDPOINT0_SIZE
#define PERIPH_EP2_BUF_BASE    (PERIPH_EP1_BUF_BASE + PERIPH_EP1_BUF_LEN)

/* rather important info unfortunately not provided by device include files: how much there is */
#define USBD_BUF_SIZE          4096

uint8_t g_hsusbd_CtrlZero, g_dev_addr=0;

enum ep_enum
{
  PERIPH_EPA = 0,
  PERIPH_EPB = 1,
  PERIPH_EPC = 2,
  PERIPH_EPD = 3,
  PERIPH_EPE = 4,
  PERIPH_EPF = 5,
  PERIPH_EPG = 6,
  PERIPH_EPH = 7,
  PERIPH_EPI = 8,
  PERIPH_EPJ = 9,
  PERIPH_EPK = 10,
  PERIPH_EPL = 11,
  PERIPH_MAX_EP,
};

static const uint8_t epcfg_eptype_table[] =
{
  [TUSB_XFER_CONTROL]     = 0, /* won't happen, since control EPs have dedicated registers */
  [TUSB_XFER_ISOCHRONOUS] = 3 << HSUSBD_EPCFG_EPTYPE_Pos,
  [TUSB_XFER_BULK]        = 1 << HSUSBD_EPCFG_EPTYPE_Pos,
  [TUSB_XFER_INTERRUPT]   = 2 << HSUSBD_EPCFG_EPTYPE_Pos,
};

static const uint8_t eprspctl_eptype_table[] =
{
  [TUSB_XFER_CONTROL]     = 0, /* won't happen, since control EPs have dedicated registers */
  [TUSB_XFER_ISOCHRONOUS] = HSUSBD_EP_RSPCTL_MODE_FLY, /* Fly Mode */
  [TUSB_XFER_BULK]        = HSUSBD_EP_RSPCTL_MODE_AUTO, /* Auto-Validate Mode */
  [TUSB_XFER_INTERRUPT]   = HSUSBD_EP_RSPCTL_MODE_MANUAL, /* Manual-Validate Mode */
};

/* reset by dcd_init(), this is used by dcd_edpt_open() to assign USBD peripheral buffer addresses */
static uint32_t bufseg_addr;

/* used by dcd_edpt_xfer() and the ISR to reset the data sync (DATA0/DATA1) in an EP0_IN transfer */
static bool active_ep0_xfer;

/* RAM table needed to track ongoing transfers performed by dcd_edpt_xfer(), dcd_in_xfer(), and the ISR */
static struct xfer_ctl_t
{
  uint8_t *data_ptr;         /* data_ptr tracks where to next copy data to (for OUT) or from (for IN) */
  // tu_fifo_t * ff; // TODO support dcd_edpt_xfer_fifo API
  union {
    uint16_t in_remaining_bytes; /* for IN endpoints, we track how many bytes are left to transfer */
    uint16_t out_bytes_so_far;   /* but for OUT endpoints, we track how many bytes we've transferred so far */
  };
  uint16_t max_packet_size;  /* needed since device driver only finds out this at runtime */
  uint16_t total_bytes;      /* quantity needed to pass as argument to dcd_event_xfer_complete() (for IN endpoints) */
  uint8_t ep_addr;
  bool dma_requested;
} xfer_table[PERIPH_MAX_EP];

/* in addition to xfer_table, additional bespoke bookkeeping is maintained for control EP0 IN */
static struct
{
  uint8_t *data_ptr;
  uint16_t in_remaining_bytes;
  uint16_t total_bytes;
} ctrl_in_xfer;

static volatile struct xfer_ctl_t *current_dma_xfer;

/*
  local helper functions
*/

static void usb_attach(void)
{
//  HSUSBD->OPER = HSUSBD_OPER_HISPDEN_Msk;   /* high-speed */
  HSUSBD->OPER &= ~HSUSBD_OPER_HISPDEN_Msk;   /* full-speed */
  HSUSBD->PHYCTL |= HSUSBD_PHYCTL_DPPUEN_Msk;
}

static void usb_detach(void)
{
  HSUSBD->OPER &= ~HSUSBD_OPER_HISPDEN_Msk;   /* full-speed */
  HSUSBD->PHYCTL &= ~HSUSBD_PHYCTL_DPPUEN_Msk;
}

//static inline void usb_memcpy(uint8_t *dest, uint8_t *src, uint16_t size)
//{
//  while(size--) *dest++ = *src++;
//}

static void usb_control_send_zlp(void)
{
  /* Status Stage */
  HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
  HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
  HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
}

/* reconstruct ep_addr from particular USB Configuration Register */
static uint8_t decode_ep_addr(HSUSBD_EP_T *ep)
{
  uint8_t ep_addr = ep->EPCFG & HSUSBD_EPCFG_EPNUM_Msk;
  if ( HSUSBD_EP_CFG_DIR_IN == (ep->EPCFG & HSUSBD_EPCFG_EPDIR_Msk) )
    ep_addr |= TUSB_DIR_IN_MASK;
  return ep_addr;
}

/* map 8-bit ep_addr into peripheral endpoint index (PERIPH_EP0...) */
static HSUSBD_EP_T *ep_entry(uint8_t ep_addr, bool add)
{
  HSUSBD_EP_T *ep;
  enum ep_enum ep_index;
  struct xfer_ctl_t *xfer;

  for (ep_index = PERIPH_EPA, xfer = &xfer_table[PERIPH_EPA], ep = HSUSBD->EP; ep_index < PERIPH_MAX_EP; ep_index++, xfer++, ep++)
  {
    if (add)
    {
      /* take first peripheral endpoint that is unused */
      if (0 == (ep->EPCFG & HSUSBD_EPCFG_EPEN_Msk)) return ep;
    }
    else
    {
      /* find a peripheral endpoint that matches ep_addr */
//      uint8_t candidate_ep_addr = decode_ep_addr(ep);
//      if (candidate_ep_addr == ep_addr) return ep;
        if (xfer->ep_addr == ep_addr) return ep;
    }
  }

  return NULL;
}

/* perform a non-control IN endpoint transfer; this is called by the ISR  */
static void dcd_userEP_in_xfer(struct xfer_ctl_t *xfer, HSUSBD_EP_T *ep)
{
  uint16_t const bytes_now = tu_min16(xfer->in_remaining_bytes, xfer->max_packet_size);

  /* precompute what amount of data will be left */
  xfer->in_remaining_bytes -= bytes_now;

  /*
  if there will be no more data to send, we replace the BUFEMPTYIF EP interrupt with TXPKIF;
  that way, we alert TinyUSB as soon as this last packet has been sent
  */
  if (0 == xfer->in_remaining_bytes)
  {
    ep->EPINTSTS = HSUSBD_EPINTSTS_TXPKIF_Msk;
    ep->EPINTEN = HSUSBD_EPINTEN_TXPKIEN_Msk;
  }

  /* provided buffers are thankfully 32-bit aligned, allowing most data to be transferred as 32-bit */
#if 0 // TODO support dcd_edpt_xfer_fifo API
  if (xfer->ff)
  {
    tu_fifo_read_n_const_addr_full_words(xfer->ff, (void *) (&ep->EPDAT_BYTE), bytes_now);
  }
  else
#endif
  {
    uint16_t countdown = bytes_now;
    while (countdown > 3)
    {
      uint32_t u32;
      memcpy(&u32, xfer->data_ptr, 4);

      ep->EPDAT = u32;
      xfer->data_ptr += 4; countdown -= 4;
    }

    while (countdown--) ep->EPDAT_BYTE = *xfer->data_ptr++;
  }

  /* for short packets, we must nudge the peripheral to say 'that's all folks' */
  if (bytes_now != xfer->max_packet_size) ep->EPRSPCTL = HSUSBD_EPRSPCTL_SHORTTXEN_Msk;
}

///* perform an IN endpoint transfer; this is called by dcd_edpt_xfer() and the ISR  */
//static void dcd_in_xfer(struct xfer_ctl_t *xfer, HSUSBD_EP_T *ep)
//{
//  uint16_t i,bytes_now = tu_min16(xfer->in_remaining_bytes, xfer->max_packet_size);

//#if 0 // TODO support dcd_edpt_xfer_fifo API
//  if (xfer->ff)
//  {
//    tu_fifo_read_n(xfer->ff, (void *) (USBD_BUF_BASE + ep->BUFSEG), bytes_now);
//  }
//  else
//#endif
//  {
//    // USB SRAM seems to only support byte access and memcpy could possibly do it by words
//    for(i = 0; i < bytes_now; i++)
//        ep->EPDAT_BYTE = *(xfer->data_ptr + i);
//  }

//  ep->EPTXCNT = bytes_now;
//}

/* called by dcd_init() as well as by the ISR during a USB bus reset */
static void bus_reset(void)
{
    
  /* Configure USB controller */
  /* Enable USB BUS, CEP and EPA global interrupt */
  HSUSBD_ENABLE_USB_INT(HSUSBD_GINTEN_USBIEN_Msk | HSUSBD_GINTEN_CEPIEN_Msk | HSUSBD_GINTEN_EPAIEN_Msk | HSUSBD_GINTEN_EPBIEN_Msk | HSUSBD_GINTEN_EPCIEN_Msk | HSUSBD_GINTEN_EPDIEN_Msk | HSUSBD_GINTEN_EPEIEN_Msk | HSUSBD_GINTEN_EPFIEN_Msk | HSUSBD_GINTEN_EPGIEN_Msk | HSUSBD_GINTEN_EPHIEN_Msk | HSUSBD_GINTEN_EPIIEN_Msk | HSUSBD_GINTEN_EPJIEN_Msk | HSUSBD_GINTEN_EPKIEN_Msk | HSUSBD_GINTEN_EPLIEN_Msk);
  /* Enable BUS interrupt */
  HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);

  /*****************************************************/
  /* Control endpoint */
  HSUSBD_SetEpBufAddr(CEP, 0, CFG_TUD_ENDPOINT0_SIZE);
  HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
//  HSUSBD->CEPINTEN = 0;
    
  HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RSTIF_Msk);
  HSUSBD_CLR_CEP_INT_FLAG(0x1ffc);

  /*****************************************************/
//  HID_InitForHighSpeed();

  for (enum ep_enum ep_index = PERIPH_EPA; ep_index < PERIPH_MAX_EP; ep_index++)
  {
    HSUSBD->EP[ep_index].EPCFG = 0;
    HSUSBD->EP[ep_index].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;
    xfer_table[ep_index].dma_requested = false;
  }
  HSUSBD->CEPCTL = HSUSBD_CEPCTL_FLUSH_Msk;
  HSUSBD->DMACNT = 0;
  HSUSBD->DMACTL = HSUSBD_DMACTL_DMARST_Msk;
  HSUSBD->DMACTL = 0;
  
//  /* allocate the default EP0 endpoints */
//  USBD->EP[PERIPH_EP0].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_IN;
//  USBD->EP[PERIPH_EP0].BUFSEG = PERIPH_EP0_BUF_BASE;
//  xfer_table[PERIPH_EP0].max_packet_size = PERIPH_EP0_BUF_LEN;

//  USBD->EP[PERIPH_EP1].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_OUT;
//  USBD->EP[PERIPH_EP1].BUFSEG = PERIPH_EP1_BUF_BASE;
//  xfer_table[PERIPH_EP1].max_packet_size = PERIPH_EP1_BUF_LEN;

  /* USB RAM beyond what we've allocated above is available to the user */
  bufseg_addr = CFG_TUD_ENDPOINT0_SIZE;

  /* Reset USB device address */
  HSUSBD_SET_ADDR(0);
//  g_dev_addr=0;

  current_dma_xfer = NULL;
}

///* centralized location for USBD interrupt enable bit mask */
//enum {
//  ENABLED_IRQS = USBD_INTSTS_VBDETIF_Msk | USBD_INTSTS_BUSIF_Msk | USBD_INTSTS_SETUP_Msk |
//                 USBD_INTSTS_USBIF_Msk   | (USE_SOF ? USBD_INTSTS_SOFIF_Msk : 0)
//};

/*
  M463 TinyUSB API driver implementation
*/

void dcd_init(uint8_t rhport)
{
  (void) rhport;

  /* Initial USB engine */
  HSUSBD_ENABLE_PHY();
  /* wait PHY clock ready */
  while(!(HSUSBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk));

  usb_detach();
  
  bus_reset();

//  USBD->INTSTS = ENABLED_IRQS;
//  USBD->INTEN  = ENABLED_IRQS;
    
  usb_attach();

}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
   NVIC_EnableIRQ(USBD20_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(USBD20_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;
  usb_control_send_zlp(); /* SET_ADDRESS is the one exception where TinyUSB doesn't use dcd_edpt_xfer() to generate a ZLP */
  g_dev_addr = dev_addr;
  // DCD can only set address after status for this request is complete.
  // do it at dcd_edpt0_status_complete()
}

static void remote_wakeup_delay(void)
{
  // try to delay for 1 ms
  uint32_t count = SystemCoreClock / 1000;
  while(count--) __NOP();
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
  // Enable PHY before sending Resume('K') state
  HSUSBD_ENABLE_USB();
  HSUSBD->OPER |= HSUSBD_OPER_RESUMEEN_Msk;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  (void) rhport;
  if((p_endpoint_desc->bEndpointAddress) == 0)
      while(1);
  
  HSUSBD_EP_T *ep = ep_entry(p_endpoint_desc->bEndpointAddress, true);
  TU_ASSERT(ep);

  /* mine the data for the information we need */
  int const dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);
  int const size = tu_edpt_packet_size(p_endpoint_desc);
  tusb_xfer_type_t const type = (tusb_xfer_type_t) p_endpoint_desc->bmAttributes.xfer;
  struct xfer_ctl_t *xfer = &xfer_table[ep - HSUSBD->EP];

  /* allocate buffer from USB RAM */
  ep->EPBUFST = bufseg_addr;
  bufseg_addr += size;
  ep->EPBUFEND = bufseg_addr - 1ul;
  
  TU_ASSERT(bufseg_addr <= USBD_BUF_SIZE);
  ep->EPMPS = size;
  
//  ep->EPRSPCTL = (HSUSBD_EP_RSPCTL_FLUSH | eprspctl_eptype_table[type]);
  ep->EPRSPCTL = (HSUSBD_EP_RSPCTL_FLUSH | HSUSBD_EP_RSPCTL_MODE_AUTO);

  /* construct USB Configuration Register value and then write it */
  uint32_t cfg = (uint32_t)tu_edpt_number(p_endpoint_desc->bEndpointAddress) << HSUSBD_EPCFG_EPNUM_Pos;
  if (TUSB_DIR_IN == dir)
    cfg |= HSUSBD_EPCFG_EPDIR_Msk;
  cfg |= epcfg_eptype_table[type] | HSUSBD_EPCFG_EPEN_Msk;
  ep->EPCFG = cfg;

  /* make a note of the endpoint size */
  xfer->max_packet_size = size;
  xfer->ep_addr = p_endpoint_desc->bEndpointAddress;

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void) rhport;
  struct xfer_ctl_t *xfer;
  uint8_t u8Value;
  uint16_t i,bytes_now;

  /* mine the data for the information we need */
  tusb_dir_t dir = tu_edpt_dir(ep_addr);
    
  if(ep_addr == 0x80)
  {
    if (total_bytes)
    {
      HSUSBD->CEPCTL = HSUSBD_CEPCTL_FLUSH_Msk;
      ctrl_in_xfer.data_ptr = buffer;
      ctrl_in_xfer.in_remaining_bytes = total_bytes;
      ctrl_in_xfer.total_bytes = total_bytes;
      HSUSBD->CEPINTSTS = HSUSBD_CEPINTSTS_INTKIF_Msk;
      HSUSBD->CEPINTEN = HSUSBD_CEPINTEN_INTKIEN_Msk;
    }
    else
    {
      usb_control_send_zlp();
    }
  }
  else if(ep_addr == 0x00)
  {
    if (total_bytes)
    {
      /* if TinyUSB is asking for EP0 OUT data, it is almost certainly already in the buffer */
      while (total_bytes < HSUSBD->CEPRXCNT);
      for (int count = 0; count < total_bytes; count++)
        *buffer++ = HSUSBD->CEPDAT_BYTE;

      dcd_event_xfer_complete(0, ep_addr, total_bytes, XFER_RESULT_SUCCESS, true);
    }
  }
  else
  {
    /* mine the data for the information we need */
    tusb_dir_t dir = tu_edpt_dir(ep_addr);
    HSUSBD_EP_T *ep = ep_entry(ep_addr, false);
    TU_ASSERT(ep);
    struct xfer_ctl_t *xfer = &xfer_table[ep - HSUSBD->EP];

    /* store away the information we'll needing now and later */
    xfer->data_ptr = buffer;
    // xfer->ff       = NULL; // TODO support dcd_edpt_xfer_fifo API
    xfer->in_remaining_bytes = total_bytes;
    xfer->total_bytes = total_bytes;

    if (TUSB_DIR_IN == dir)
    {
      ep->EPINTEN = HSUSBD_EPINTEN_BUFEMPTYIEN_Msk;
    }
    else
    {
      xfer->out_bytes_so_far = 0;
      ep->EPINTEN = HSUSBD_EPINTEN_RXPKIEN_Msk;
    }
  }

  return true;
}

#if 0 // TODO support dcd_edpt_xfer_fifo API
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void) rhport;

  /* mine the data for the information we need */
  tusb_dir_t dir = tu_edpt_dir(ep_addr);
  USBD_EP_T *ep = ep_entry(ep_addr, false);
  struct xfer_ctl_t *xfer = &xfer_table[ep - USBD->EP];

  /* store away the information we'll needing now and later */
  xfer->data_ptr = NULL;      // Indicates a FIFO shall be used
  xfer->ff       = ff;
  xfer->in_remaining_bytes = total_bytes;
  xfer->total_bytes = total_bytes;

  if (TUSB_DIR_IN == dir)
  {
    dcd_in_xfer(xfer, ep);
  }
  else
  {
    xfer->out_bytes_so_far = 0;
    ep->MXPLD = xfer->max_packet_size;
  }

  return true;
}
#endif

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
    
  if (tu_edpt_number(ep_addr))
  {
    HSUSBD_EP_T *ep = ep_entry(ep_addr, false);
    TU_ASSERT(ep, );
    ep->EPRSPCTL = (ep->EPRSPCTL & 0xf7) | HSUSBD_EPRSPCTL_HALT_Msk;
  }
  else
  {
    HSUSBD->CEPCTL = HSUSBD_CEPCTL_STALLEN_Msk;
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  if (tu_edpt_number(ep_addr))
  {
    HSUSBD_EP_T *ep = ep_entry(ep_addr, false);
    TU_ASSERT(ep, );
    ep->EPRSPCTL = HSUSBD_EPRSPCTL_TOGGLE_Msk;
  }
}

void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;
  __IO uint32_t IrqStL, IrqSt;
  uint32_t i, u32TimeOutCnt;
  uint8_t usbd_SetupPacket[8];
  enum ep_enum ep_index;
  uint32_t mask;
  struct xfer_ctl_t *xfer;
  HSUSBD_EP_T *ep;
  uint8_t u8Value;
  uint16_t bytes_now;

  // Mask non-enabled irqs, ex. SOF
  IrqStL = HSUSBD->GINTSTS & HSUSBD->GINTEN;
    
  /* USB interrupt */
    if(IrqStL & HSUSBD_GINTSTS_USBIF_Msk)
    {
        IrqSt = HSUSBD->BUSINTSTS & HSUSBD->BUSINTEN;

        if(IrqSt & HSUSBD_BUSINTSTS_SOFIF_Msk)
        {
            
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SOFIF_Msk);
            /* Start-Of-Frame event */
            dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
        }

        if(IrqSt & HSUSBD_BUSINTSTS_RSTIF_Msk)
        {

            bus_reset();
            
            //tusb_speed_t speed = (HSUSBD->OPER & HSUSBD_OPER_CURSPD_Msk) ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL;
            dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
            
            HSUSBD_ResetDMA();
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RSTIF_Msk);
            HSUSBD_CLR_CEP_INT_FLAG(0x1ffc);
            
        }

        if(IrqSt & HSUSBD_BUSINTSTS_RESUMEIF_Msk)
        {
            HSUSBD_ENABLE_USB();
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!(HSUSBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
                if(--u32TimeOutCnt == 0) break;

            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RESUMEIF_Msk);
            
            dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
        }

        if(IrqSt & HSUSBD_BUSINTSTS_SUSPENDIF_Msk)
        {

            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SUSPENDIF_Msk);

            /* Enable USB but disable PHY */
            HSUSBD_DISABLE_PHY();
            dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
        }

        if(IrqSt & HSUSBD_BUSINTSTS_HISPDIF_Msk)
        {
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_HISPDIF_Msk);
        }

        if(IrqSt & HSUSBD_BUSINTSTS_DMADONEIF_Msk)
        {
//            g_hsusbd_DmaDone = 1;
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_DMADONEIF_Msk);

            if(HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk)
            {
//                if(g_hsusbd_ShortPacket == 1)
//                {
//                    HSUSBD->EP[EPA].EPRSPCTL = (HSUSBD->EP[EPA].EPRSPCTL & 0x10) | HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
//                    g_hsusbd_ShortPacket = 0;
//                }
            }
        }

        if(IrqSt & HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if(IrqSt & HSUSBD_BUSINTSTS_VBUSDETIF_Msk)
        {
            if(HSUSBD_IS_ATTACHED())
            {
                /* USB Plug In */
                HSUSBD_ENABLE_USB();
            }
            else
            {
                /* USB Un-plug */
                HSUSBD_DISABLE_USB();
            }
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    if(IrqStL & HSUSBD_GINTSTS_CEPIF_Msk)
    {
        IrqSt = HSUSBD->CEPINTSTS & HSUSBD->CEPINTEN;

        if(IrqSt & HSUSBD_CEPINTSTS_SETUPTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPTKIF_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_SETUPPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPPKIF_Msk);
//            HSUSBD_ProcessSetupPacket();
            /* get SETUP packet from USB buffer */
            usbd_SetupPacket[0] = (uint8_t)(HSUSBD->SETUP1_0 & 0xfful);
            usbd_SetupPacket[1] = (uint8_t)((HSUSBD->SETUP1_0 >> 8) & 0xfful);
            usbd_SetupPacket[2] = (uint8_t)(HSUSBD->SETUP3_2 & 0xfful);
            usbd_SetupPacket[3] = (uint8_t)((HSUSBD->SETUP3_2 >> 8) & 0xfful);
            usbd_SetupPacket[4] = (uint8_t)(HSUSBD->SETUP5_4 & 0xfful);
            usbd_SetupPacket[5] = (uint8_t)((HSUSBD->SETUP5_4 >> 8) & 0xfful);
            usbd_SetupPacket[6] = (uint8_t)(HSUSBD->SETUP7_6 & 0xfful);
            usbd_SetupPacket[7] = (uint8_t)((HSUSBD->SETUP7_6 >> 8) & 0xfful);
            dcd_event_setup_received(0, (uint8_t *)usbd_SetupPacket, true);
//            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
//            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
//            g_hsusbd_CtrlZero = (uint8_t)0ul;
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_OUTTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_OUTTKIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_INTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
            
            HSUSBD->CEPINTSTS = HSUSBD_CEPINTSTS_TXPKIF_Msk;

            if (!(IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk))
            {
                HSUSBD->CEPINTEN = HSUSBD_CEPINTEN_TXPKIEN_Msk;
                uint16_t bytes_now = tu_min16(ctrl_in_xfer.in_remaining_bytes, CFG_TUD_ENDPOINT0_SIZE);
                for (int count = 0; count < bytes_now; count++)
                  HSUSBD->CEPDAT_BYTE = *ctrl_in_xfer.data_ptr++;
                ctrl_in_xfer.in_remaining_bytes -= bytes_now;
                HSUSBD_START_CEP_IN(bytes_now);
            }
            else
            {
                HSUSBD->CEPINTEN = HSUSBD_CEPINTEN_TXPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk;
            }
            
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_PINGIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_PINGIF_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_TXPKIF_Msk)
        {
            
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);

            /* alert TinyUSB that the EP0 IN transfer has finished */
            if ( (0 == ctrl_in_xfer.in_remaining_bytes) || (0 == ctrl_in_xfer.total_bytes) )
              dcd_event_xfer_complete(0, 0x80, ctrl_in_xfer.total_bytes, XFER_RESULT_SUCCESS, true);

            if (ctrl_in_xfer.in_remaining_bytes)
            {
              HSUSBD->CEPINTSTS = HSUSBD_CEPINTSTS_INTKIF_Msk;
              HSUSBD->CEPINTEN = HSUSBD_CEPINTEN_INTKIEN_Msk;
            }
            else
            {
              /* TinyUSB does its own fragmentation and ZLP for EP0; a transfer of zero means a ZLP */
              if (0 == ctrl_in_xfer.total_bytes) HSUSBD->CEPCTL = HSUSBD_CEPCTL_ZEROLEN_Msk;

              HSUSBD->CEPINTSTS = HSUSBD_CEPINTSTS_STSDONEIF_Msk;
              HSUSBD->CEPINTEN = HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk;
            }
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_RXPKIF_Msk)
        {
//            dcd_event_xfer_complete(0, 0x00, 0, XFER_RESULT_SUCCESS, true);
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_RXPKIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_NAKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_NAKIF_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_STALLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STALLIF_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_ERRIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_ERRIF_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk)
        {
//            HSUSBD_UpdateDeviceState();
//            if((g_dev_addr != HSUSBD->FADDR) && (HSUSBD->FADDR == 0))
//                HSUSBD_SET_ADDR(g_dev_addr);
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            /* given ACK from host has happened, we can now set the address (if not already done) */
            if((HSUSBD->FADDR != g_dev_addr) && (HSUSBD->FADDR == 0))
            {
              HSUSBD->FADDR = g_dev_addr;

              for (enum ep_enum ep_index = PERIPH_EPA; ep_index < PERIPH_MAX_EP; ep_index++)
              {
                if (HSUSBD->EP[ep_index].EPCFG & HSUSBD_EPCFG_EPEN_Msk) HSUSBD->EP[ep_index].EPRSPCTL = HSUSBD_EPRSPCTL_TOGGLE_Msk;
              }
            }
            HSUSBD->CEPINTEN = HSUSBD_CEPINTEN_SETUPPKIEN_Msk;
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_BUFFULLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFFULLIF_Msk);
            return;
        }

        if(IrqSt & HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk);
            return;
        }
    }
    
    if (IrqStL & (HSUSBD_GINTSTS_EPAIF_Msk | HSUSBD_GINTSTS_EPBIF_Msk | HSUSBD_GINTSTS_EPCIF_Msk | HSUSBD_GINTSTS_EPDIF_Msk | HSUSBD_GINTSTS_EPEIF_Msk | HSUSBD_GINTSTS_EPFIF_Msk | HSUSBD_GINTSTS_EPGIF_Msk | HSUSBD_GINTSTS_EPHIF_Msk | HSUSBD_GINTSTS_EPIIF_Msk | HSUSBD_GINTSTS_EPJIF_Msk | HSUSBD_GINTSTS_EPKIF_Msk | HSUSBD_GINTSTS_EPLIF_Msk))
    {
        /* service PERIPH_EPA through PERIPH_EPL */
        for (ep_index = PERIPH_EPA, mask = HSUSBD_GINTSTS_EPAIF_Msk, xfer = &xfer_table[PERIPH_EPA], ep = &HSUSBD->EP[PERIPH_EPA]; ep_index < PERIPH_MAX_EP; ep_index++, mask <<= 1, xfer++, ep++)
        {
          if(IrqStL & mask)
          {
            uint8_t const ep_addr = xfer->ep_addr;
            bool const out_ep = !(ep_addr & TUSB_DIR_IN_MASK);
            uint32_t ep_state = ep->EPINTSTS & ep->EPINTEN;

            if (out_ep)
            {
#if USE_DMA
              xfer->dma_requested = true;
              service_dma();
#else
              uint16_t const available_bytes = ep->EPDATCNT & HSUSBD_EPDATCNT_DATCNT_Msk;
              /* copy the data from the PC to the previously provided buffer */
#if 0 // TODO support dcd_edpt_xfer_fifo API
              if (xfer->ff)
              {
                tu_fifo_write_n_const_addr_full_words(xfer->ff, (const void *) &ep->EPDAT_BYTE, tu_min16(available_bytes, xfer->total_bytes - xfer->out_bytes_so_far));
              }
              else
#endif
              {
                for (int count = 0; (count < available_bytes) && (xfer->out_bytes_so_far < xfer->total_bytes); count++, xfer->out_bytes_so_far++)
                {
                  *xfer->data_ptr++ = ep->EPDAT_BYTE;
                }
              }

              /* when the transfer is finished, alert TinyUSB; otherwise, continue accepting more data */
              if ( (xfer->total_bytes == xfer->out_bytes_so_far) || (available_bytes < xfer->max_packet_size) )
              {
                dcd_event_xfer_complete(0, ep_addr, xfer->out_bytes_so_far, XFER_RESULT_SUCCESS, true);
              }
#endif

            }
            else if (ep_state & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
            {
              /* send any remaining data */
              dcd_userEP_in_xfer(xfer, ep);
            }
            else if (ep_state & HSUSBD_EPINTSTS_TXPKIF_Msk)
            {
              /* alert TinyUSB that we've finished */
              dcd_event_xfer_complete(0, ep_addr, xfer->total_bytes, XFER_RESULT_SUCCESS, true);
              ep->EPINTEN = 0;
            }

            ep->EPINTSTS = ep_state;
          }
        }
    }
    

}

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
//void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
//{
//  (void) rhport;

//  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
//      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
//      request->bRequest == TUSB_REQ_SET_ADDRESS )
//  {
//    uint8_t const dev_addr = (uint8_t) request->wValue;

//    // Setting new address after the whole request is complete
//    HSUSBD_SET_ADDR(dev_addr);
//  }
//}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  usb_detach();
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  usb_attach();
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

#endif

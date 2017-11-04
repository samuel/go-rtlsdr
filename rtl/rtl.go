// Package rtl provides bindings to the rtl-sdr library.
package rtl

// #cgo LDFLAGS: -lrtlsdr
// #cgo darwin CFLAGS: -I/usr/local/include
// #cgo darwin LDFLAGS: -L/usr/local/lib
// #cgo pkg-config: libusb-1.0
// #include <stdlib.h>
// #include <rtl-sdr.h>
// #include <libusb.h>
// #include "exports.h"
import "C"

import (
	"errors"
	"fmt"
	"log"
	"runtime"
	"sync"
	"unsafe"
)

// Possible errors.
var (
	ErrFailed           = errors.New("rtl: operation failed")
	ErrNoDevices        = errors.New("rtl: no devices")
	ErrNoMatchingDevice = errors.New("rtl: no matching device")
)

type TunerType int

// Known tuner types
const (
	TunerTypeUnknown TunerType = C.RTLSDR_TUNER_UNKNOWN
	TunerTypeE4000   TunerType = C.RTLSDR_TUNER_E4000
	TunerTypeFC0012  TunerType = C.RTLSDR_TUNER_FC0012
	TunerTypeFC0013  TunerType = C.RTLSDR_TUNER_FC0013
	TunerTypeFC2580  TunerType = C.RTLSDR_TUNER_FC2580
	TunerTypeR820T   TunerType = C.RTLSDR_TUNER_R820T
	TunerTypeR828D   TunerType = C.RTLSDR_TUNER_R828D
)

var (
	tunerTypeNames = map[TunerType]string{
		TunerTypeUnknown: "Unknown",
		TunerTypeE4000:   "E4000",
		TunerTypeFC0012:  "FC0012",
		TunerTypeFC0013:  "FC0013",
		TunerTypeFC2580:  "FC2580",
		TunerTypeR820T:   "R820T",
		TunerTypeR828D:   "R828D",
	}
)

// LibUSBErr is an error from libusb
type LibUSBErr int

func (e LibUSBErr) Error() string {
	return C.GoString(C.libusb_error_name(C.int(e)))
}

func (tt TunerType) String() string {
	if name := tunerTypeNames[tt]; name == "" {
		return "Other"
	} else {
		return name
	}
}

// AsyncCallback returns true to stop the async loop
type AsyncCallback func(buf []byte) bool

type asyncCallbackContext struct {
	cb  AsyncCallback
	dev *Device
}

//export cbAsyncGo
func cbAsyncGo(buf *C.uchar, size C.uint32_t, ctx unsafe.Pointer) {
	cbHandlesMu.RLock()
	cbc := cbHandles[uintptr(ctx)]
	cbHandlesMu.RUnlock()

	goBuf := (*[1 << 30]byte)(unsafe.Pointer(buf))[:size:size]
	if cbc.cb(goBuf) {
		cbHandlesMu.Lock()
		C.rtlsdr_cancel_async(cbc.dev.cDev)
		cbHandles[uintptr(ctx)] = nil
		cbHandlesMu.Unlock()
	}
}

type Device struct {
	cDev *C.rtlsdr_dev_t
}

// DeviceCount returns the number of rtl-sdr devices found.
func DeviceCount() int {
	return int(C.rtlsdr_get_device_count())
}

// DeviceName returns the name of a device by index
func DeviceName(index int) string {
	cName := C.rtlsdr_get_device_name(C.uint32_t(index))
	return C.GoString(cName)
}

// DeviceUSBStrings returns the USB device strings.
func DeviceUSBStrings(index int) (manufacturer, productName, serialNumber string, err error) {
	var manufact [256]C.char
	var prod [256]C.char
	var ser [256]C.char
	if C.rtlsdr_get_device_usb_strings(C.uint32_t(index), (*C.char)(&manufact[0]), (*C.char)(&prod[0]), (*C.char)(&ser[0])) != 0 {
		return "", "", "", ErrFailed
	}
	return C.GoString((*C.char)(&manufact[0])), C.GoString((*C.char)(&prod[0])), C.GoString((*C.char)(&ser[0])), nil
}

// IndexBySerial returns device index by USB serial string descriptor.
func IndexBySerial(serial string) (int, error) {
	cSerial := C.CString(serial)
	defer C.free(unsafe.Pointer(cSerial))
	res := C.rtlsdr_get_index_by_serial(cSerial)
	if res < 0 {
		switch res {
		case -1:
			// Name is NULL. Shouldn't ever happen.
			return -1, ErrFailed
		case -2:
			return -1, ErrNoDevices
		case -3:
			return -1, ErrNoMatchingDevice
		default:
			return -1, ErrFailed
		}
	}
	return int(res), nil
}

// Open opens an rtl-sdr device by index.
func Open(index int) (*Device, error) {
	dev := &Device{}
	if C.rtlsdr_open(&dev.cDev, C.uint32_t(index)) < 0 {
		return nil, ErrFailed
	}
	runtime.SetFinalizer(dev, func(dev *Device) { dev.Close() })
	return dev, nil
}

// Close closes the device.
func (dev *Device) Close() error {
	if dev.cDev != nil {
		if C.rtlsdr_close(dev.cDev) < 0 {
			return ErrFailed
		}
		dev.cDev = nil
	}
	return nil
}

// SetXtalFreq sets the crystal oscillator frequencies used for the RTL2832 and the tuner IC.
//
// Usually both ICs use the same clock. Changing the clock may make sense if
// you are applying an external clock to the tuner or to compensate the
// frequency (and samplerate) error caused by the original (cheap) crystal.
//
// NOTE: Call this function only if you fully understand the implications.
//
// rtl_freq is the frequency value used to clock the RTL2832 in Hz
// tuner_freq is the frequency value used to clock the tuner IC in Hz
func (dev *Device) SetXtalFreq(rtlFreq, tunerFreq uint) error {
	if C.rtlsdr_set_xtal_freq(dev.cDev, C.uint32_t(rtlFreq), C.uint32_t(tunerFreq)) != 0 {
		return ErrFailed
	}
	return nil
}

// XtalFreq returns the crystal oscillator frequencies used for the RTL2832 and the tuner IC.
//
// Usually both ICs use the same clock.
//
// Returns frequency value used to clock the RTL2832 in Hz and
// frequency value used to clock the tuner IC in Hz.
func (dev *Device) XtalFreq() (uint, uint, error) {
	var rtlFreq, tunerFreq C.uint32_t
	if C.rtlsdr_get_xtal_freq(dev.cDev, &rtlFreq, &tunerFreq) != 0 {
		return 0, 0, ErrFailed
	}
	return uint(rtlFreq), uint(tunerFreq), nil
}

// CenterFreq returns the actual frequency the device is tuned to in Hz.
func (dev *Device) CenterFreq() (uint, error) {
	freq := C.rtlsdr_get_center_freq(dev.cDev)
	if freq == 0 {
		return 0, ErrFailed
	}
	return uint(freq), nil
}

// SetCenterFreq tunes to to the provided center frequency in hz.
func (dev *Device) SetCenterFreq(freqHz uint) error {
	if C.rtlsdr_set_center_freq(dev.cDev, C.uint32_t(freqHz)) != 0 {
		return ErrFailed
	}
	return nil
}

// TunerType returns the type of the tuner device.
func (dev *Device) TunerType() TunerType {
	return TunerType(C.rtlsdr_get_tuner_type(dev.cDev))
}

// TunerGains returns a list of gains supported by the tuner.
func (dev *Device) TunerGains() ([]int, error) {
	nGains := C.rtlsdr_get_tuner_gains(dev.cDev, nil)
	if nGains <= 0 {
		return nil, ErrFailed
	}
	cGains := make([]C.int, nGains)
	C.rtlsdr_get_tuner_gains(dev.cDev, &cGains[0])
	gains := make([]int, nGains)
	for i := 0; i < len(gains); i++ {
		gains[i] = int(cGains[i])
	}
	return gains, nil
}

// SetTunerGain sets the gain for the device. Manual gain mode must be enabled for this to work.
// Value is in tenths of dB and varies by tuner. For the E4000 tuner: -10, 15, 40, 65, 90, 115, 140, 165,
// 190, 215, 240, 290, 340, 420, 430, 450, 470, 490 Gain values are in tenths of dB, e.g. 115 means 11.5 dB.
// Should use TunerGains() to get the list of supported gains.
func (dev *Device) SetTunerGain(gain int) error {
	if C.rtlsdr_set_tuner_gain(dev.cDev, C.int(gain)) < 0 {
		return ErrFailed
	}
	return nil
}

// TunerGain returns the actual gain the device is configured to in tength of a dB (115 means 11.5 dB)
func (dev *Device) TunerGain() (int, error) {
	gain := C.rtlsdr_get_tuner_gain(dev.cDev)
	if gain == 0 {
		return 0, ErrFailed
	}
	return int(gain), nil
}

// SetTunerIfGain sets the intermediate frequency gain for the device.
// - stage: intermediate frequency gain stage number (1 to 6 for E4000)
// - gain: tenths of a dB, -30 means -3.0 dB
func (dev *Device) SetTunerIfGain(stage, gain int) error {
	if C.rtlsdr_set_tuner_if_gain(dev.cDev, C.int(stage), C.int(gain)) < 0 {
		return ErrFailed
	}
	return nil
}

// SetTunerGainMode sets the gain mode (automatic/manual) for the device.
// Manual gain mode must be enabled for the gain setter function to work.
func (dev *Device) SetTunerGainMode(manual bool) error {
	cManual := C.int(0)
	if manual {
		cManual = 1
	}
	if C.rtlsdr_set_tuner_gain_mode(dev.cDev, cManual) != 0 {
		return ErrFailed
	}
	return nil
}

// SetAGCMode enables or disables the internal digital AGC of the RTL2832.
func (dev *Device) SetAGCMode(enabled bool) error {
	cEnabled := C.int(0)
	if enabled {
		cEnabled = 1
	}
	if C.rtlsdr_set_agc_mode(dev.cDev, cEnabled) != 0 {
		return ErrFailed
	}
	return nil
}

// SetSampleRate selects the baseband filters according to the requested sample rate
func (dev *Device) SetSampleRate(rate uint) error {
	if C.rtlsdr_set_sample_rate(dev.cDev, C.uint32_t(rate)) != 0 {
		return ErrFailed
	}
	return nil
}

// SampleRate returns the actual sample rate the device is configured to
func (dev *Device) SampleRate() (int, error) {
	sampleRate := C.rtlsdr_get_sample_rate(dev.cDev)
	if sampleRate == 0 {
		return 0, ErrFailed
	}
	return int(sampleRate), nil
}

// ResetBuffer resets internal buffers.
func (dev *Device) ResetBuffer() error {
	if C.rtlsdr_reset_buffer(dev.cDev) < 0 {
		return ErrFailed
	}
	return nil
}

type DirectSampling int

const (
	DirectSamplingOff  DirectSampling = 0
	DirectSamplingIADC DirectSampling = 1
	DirectSamplingQADC DirectSampling = 2
)

func (ds DirectSampling) String() string {
	switch ds {
	case DirectSamplingOff:
		return "Off"
	case DirectSamplingIADC:
		return "I-ADC"
	case DirectSamplingQADC:
		return "Q-ADC"
	}
	return fmt.Sprintf("DirectSampling(%d)", ds)
}

// SetDirectSampling enables or disables the direct sampling mode. When enabled, the IF mode
// of the RTL2832 is activated, and rtlsdr_set_center_freq() will control
// the IF-frequency of the DDC, which can be used to tune from 0 to 28.8 MHz
// (xtal frequency of the RTL2832).
func (dev *Device) SetDirectSampling(ds DirectSampling) error {
	if C.rtlsdr_set_direct_sampling(dev.cDev, C.int(ds)) < 0 {
		return ErrFailed
	}
	return nil
}

// GetDirectSampling returns the state of the direct sampling mode.
func (dev *Device) GetDirectSampling() (DirectSampling, error) {
	r := C.rtlsdr_get_direct_sampling(dev.cDev)
	if r < 0 {
		return 0, ErrFailed
	}
	return DirectSampling(r), nil
}

// SetOffsetTuning enables or disables offset tuning for zero-IF tuners, which allows to avoid
// problems caused by the DC offset of the ADCs and 1/f noise.
func (dev *Device) SetOffsetTuning(enabled bool) error {
	var e C.int
	if enabled {
		e = 1
	}
	if C.rtlsdr_set_offset_tuning(dev.cDev, e) < 0 {
		return ErrFailed
	}
	return nil
}

// GetOffsetTuning returns the state of the offset tuning mode
func (dev *Device) GetOffsetTuning() (bool, error) {
	r := C.rtlsdr_get_offset_tuning(dev.cDev)
	if r < 0 {
		return false, ErrFailed
	}
	return r == 1, nil
}

// SetBiasTee enables or disables the bias tee on GPIO PIN 0.
func (dev *Device) SetBiasTee(enabled bool) error {
	var e C.int
	if enabled {
		e = 1
	}
	if C.rtlsdr_set_bias_tee(dev.cDev, e) < 0 {
		return ErrFailed
	}
	return nil
}

// Read performs a synchronous read from the device.
func (dev *Device) Read(buf []byte) (int, error) {
	var nRead C.int
	if res := C.rtlsdr_read_sync(dev.cDev, unsafe.Pointer(&buf[0]), C.int(len(buf)), &nRead); res != 0 {
		return 0, LibUSBErr(int(res))
	}
	return int(nRead), nil
}

type buffer struct {
	bytes []byte
	size  int
}

// ReadAsyncUsingSync starts an asynchronous background read. The provided callback
// is called for all buffers read. This is similar to ReadAsync but this version
// starts a goroutine that uses the synhcornous read of the librtlsdr library.
func (dev *Device) ReadAsyncUsingSync(nBuffers, bufferSize int, cb AsyncCallback) error {
	bufferSize &^= 1
	bufferCache := make(chan buffer, nBuffers)
	sampleChan := make(chan buffer, nBuffers)

	for i := 0; i < nBuffers; i++ {
		bufferCache <- buffer{bytes: make([]byte, bufferSize)}
	}

	go func() {
		for {
			buf, ok := <-sampleChan
			if !ok {
				close(bufferCache)
				cb(nil)
				break
			}
			if cb(buf.bytes[:buf.size]) {
				close(bufferCache)
				break
			}
			bufferCache <- buf
		}
	}()

	go func() {
		for {
			buf, ok := <-bufferCache
			if !ok {
				break
			}
			n, err := dev.Read(buf.bytes)
			if err != nil {
				close(sampleChan)
				break
			}
			select {
			case sampleChan <- buffer{bytes: buf.bytes, size: n}:
			default:
				log.Print("dropped packet")
			}
		}
	}()

	return nil
}

var (
	cbHandlesMu   sync.RWMutex
	cbHandlesNext uintptr
	cbHandles     = make(map[uintptr]*asyncCallbackContext)
)

// ReadAsyncUsingSync starts an asynchronous background read. The provided callback
// is called for all buffers read. This is similar to ReadAsyncUsingSync but this version
// relies on the built-in asynchronous reading in the librtlsdr library.
func (dev *Device) ReadAsync(nBuffers, bufferSize int, cb AsyncCallback) error {
	go func() {
		cbHandlesMu.Lock()
		cbHandlesNext++
		handle := cbHandlesNext
		cbHandles[handle] = &asyncCallbackContext{
			cb:  cb,
			dev: dev,
		}
		cbHandlesMu.Unlock()
		C.read_async(dev.cDev, (*[0]byte)(unsafe.Pointer(C.cbAsyncPtr)), C.intptr_t(handle), C.uint32_t(nBuffers), C.uint32_t(bufferSize))
	}()
	return nil
}

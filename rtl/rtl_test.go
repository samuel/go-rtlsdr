package rtl

import (
	"testing"
)

func TestBasics(t *testing.T) {
	devCount := DeviceCount()
	if devCount < 0 || devCount > 16 {
		t.Fatal("DeviceCount failed")
	}
	t.Logf("Device count: %d", devCount)

	for i := 0; i < devCount; i++ {
		name := DeviceName(i)
		if name == "" {
			t.Fatalf("Failed to get device name for index %d", i)
		}

		manufact, product, serial, err := DeviceUSBStrings(i)
		if err != nil {
			t.Fatalf("GetDeviceUSBStrings failed: %+v", err)
		}

		t.Logf("Device %d: %s", i, name)
		t.Logf("\tManufacturer: %s", manufact)
		t.Logf("\tProduct: %s", product)
		t.Logf("\tSerial: %s", serial)

		dev, err := Open(i)
		if err != nil {
			t.Fatalf("Open failed: %+v", err)
		}
		t.Logf("\tTuner Type: %s", dev.TunerType())
		if gains, err := dev.TunerGains(); err != nil {
			t.Errorf("TunerGains failed: %+v", err)
		} else {
			t.Logf("\tTuner Gains: %+v", gains)
		}

		if err := dev.SetCenterFreq(97.9e6); err != nil {
			t.Errorf("Failed to set center freq: %s", err)
		}

		// buf := make([]byte, 1024)
		// n, err := dev.Read(buf)
		// if err != nil {
		// 	t.Errorf("Read failed: %s", err)
		// }
		// t.Logf("Read %d bytes synchronously\n", n)
		if err := dev.ResetBuffer(); err != nil {
			t.Fatalf("Failed to reset internal buffers: %s", err)
		}
		ch := make(chan int, 1)
		if err := dev.ReadAsync(1, 32768, func(buf []byte) bool {
			ch <- len(buf)
			return true
		}); err != nil {
			t.Errorf("Async read failed: %s", err)
		}
		nRead := <-ch
		t.Logf("Read %d bytes asynchronously\n", nRead)
		if err := dev.Close(); err != nil {
			t.Fatalf("Close failed: %+v", err)
		}
	}
}

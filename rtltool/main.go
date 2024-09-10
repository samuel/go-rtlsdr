package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"runtime/pprof"

	"github.com/samuel/go-dsp/dsp"
	"github.com/samuel/go-rtlsdr/rtl"
)

const (
	nBuffers   = 15
	bufferSize = 256 * 1024 // in samples
)

var flagCPUProfile = flag.Bool("profile.cpu", false, "Enable CPU profiling")

type buffer struct {
	bytes []byte
	size  int
}

func main() {
	flag.Parse()

	if *flagCPUProfile {
		wr, err := os.Create("cpu.prof")
		if err != nil {
			log.Fatal(err)
		}
		defer wr.Close()

		if err := pprof.StartCPUProfile(wr); err != nil {
			log.Fatal(err)
		}
	}

	dev, err := rtl.Open(0)
	if err != nil {
		log.Fatalf("Failed to open device: %s", err.Error())
	}
	defer dev.Close()

	sampleRate := 170000
	outputRate := 32000
	postDownsample := 4

	sampleRate *= postDownsample
	frequency := int(92.7e6)
	downsample := 1000000/sampleRate + 1
	captureRate := downsample * sampleRate
	captureFreq := frequency + 16000 + captureRate/4

	fmt.Fprintf(os.Stderr, "Oversampling input by: %dx\n", downsample)
	fmt.Fprintf(os.Stderr, "Oversampling output by: %dx\n", postDownsample)
	fmt.Fprintf(os.Stderr, "Sampling at %d Hz\n", captureRate)
	fmt.Fprintf(os.Stderr, "Tuned to %d Hz\n", captureFreq)

	if err := dev.SetSampleRate(uint(captureRate)); err != nil {
		log.Fatalf("Failed to set sample rate: %s", err.Error())
	}

	if err := dev.SetCenterFreq(uint(captureFreq)); err != nil {
		log.Fatalf("Failed to set center freq: %s", err.Error())
	}

	if err := dev.ResetBuffer(); err != nil {
		log.Fatalf("Failed to reset buffers: %s", err.Error())
	}

	rotate90 := &dsp.Rotate90Filter{}
	lowPass1 := &dsp.LowPassDownsampleComplexFilter{Downsample: downsample}
	fmDemod := &dsp.FMDemodFilter{}
	// lowPass2 := &dsp.LowPassDownsampleRationalFilter{Fast: postDownsample, Slow: 1}
	// lowPass3 := &dsp.LowPassDownsampleRationalFilter{Fast: sampleRate / postDownsample, Slow: outputRate}
	lowPass2 := &dsp.LowPassDownsampleRationalFilter{Fast: sampleRate, Slow: outputRate}

	stopChan := make(chan bool)

	bytes := make([]byte, bufferSize*2)
	samples := make([]complex64, bufferSize)
	pcm := make([]float32, bufferSize)

	_ = dev.ReadAsync(nBuffers, bufferSize, func(buf []byte) bool {
		select {
		case <-stopChan:
			return true
		default:
		}

		n := len(buf)
		n /= 2
		dsp.Ui8toc64(buf, samples[:n])

		var samples2 []complex64
		samples2 = rotate90.Filter(samples[:n])
		samples2 = lowPass1.Filter(samples2)
		n = fmDemod.Demodulate(samples2, pcm)
		pcm2 := lowPass2.Filter(pcm[:n])
		// if pcm2, err = lowPass3.Filter(pcm2); err != nil {
		// 	log.Fatal(err)
		// }
		dsp.F32toi16ble(pcm2, bytes, 1<<14)
		if _, err := os.Stdout.Write(bytes[:len(pcm2)*2]); err != nil {
			log.Fatal(err)
		}

		return false
	})

	signalChan := make(chan os.Signal, 1)
	signal.Notify(signalChan, os.Interrupt)
	<-signalChan
	close(stopChan)

	if *flagCPUProfile {
		pprof.StopCPUProfile()
	}
}

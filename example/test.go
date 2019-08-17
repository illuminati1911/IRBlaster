package main
/*
 * This example uses direct access to character device file
 * instead of the C user mode library.
 */


import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"os"
)

type iRBlasterConfig struct {
	LeadingPulseWidth uint32
	LeadingGapWidth   uint32
	OnePulseWidth uint32
	OneGapWidth uint32
	ZeroPulseWidth uint32
	ZeroGapWidth uint32
	TrailingPulseWidth uint32
	Frequency uint32
	DCN uint32
	DCM uint32
	code [0x200]byte
}

func main() {
	var b [0x200]byte
	code := "11001101001100100000111111110000"
	copy(b[:], code)
	t := iRBlasterConfig{
		LeadingPulseWidth: 9000,
		LeadingGapWidth:   4500,
		OnePulseWidth: 560,
		OneGapWidth: 1680,
		ZeroPulseWidth: 560,
		ZeroGapWidth: 560,
		TrailingPulseWidth: 560,
		Frequency: 38000,
		DCN: 50,
		DCM: 100,
		code: b,
	}
	buf := &bytes.Buffer{}
	err := binary.Write(buf, binary.LittleEndian, t)
	if err != nil {
		panic(err)
	}
	fmt.Println(buf.Bytes())

	f, err := os.OpenFile("/dev/irblaster", os.O_WRONLY, 0600)
	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()
	f.Write(buf.Bytes())
}

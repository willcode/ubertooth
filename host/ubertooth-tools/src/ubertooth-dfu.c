/* -*- c -*- */
/*
 * Copyright 2015 Dominic Spill
 *
 * This file is part of Project Ubertooth
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libbtbb; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <getopt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>  // sleep
#include "ubertooth.h"
#include "dfu.h"

/*
 * DFU suffix / signature / CRC
 */

/* CRC32 implementation for DFU suffix */
static uint32_t crc32(uint8_t *data, uint32_t data_len) {
	uint32_t crc = 0xffffffff;
	uint32_t i;
	for(i=0; i<data_len; i++)
		crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0xff];
	return crc;
}

static int check_suffix(FILE* signedfile, DFU_suffix* suffix) {
	uint8_t *data;
	uint32_t crc, data_length;
	long file_length;

	printf("Checking firmware signature\n");
	if (fseek(signedfile, 0, SEEK_END) != 0 ||
	    (file_length = ftell(signedfile)) < 0) {
		perror("DFU file");
		return 1;
	}
	if (file_length < (long)sizeof(DFU_suffix)) {
		fprintf(stderr, "DFU file too short (%ld bytes)\n", file_length);
		return 1;
	}
	data_length = file_length - 4; // Ignore 4 byte CRC
	if (fseek(signedfile, -16, SEEK_END) != 0 || // Start of DFU suffix
	    fread(suffix, 1, 16, signedfile) != 16) {
		perror("DFU file");
		return 1;
	}

	if(suffix->bLength != 16) {
		fprintf(stderr, "Unknown DFU suffix length: %d\n", suffix->bLength);
		return 1;
	}

	// We only know about dfu version 1.0/1.1
	// This needs to be smarter to support other versions if/when they exist
	if((suffix->bcdDFU != 0x0100) && (suffix->bcdDFU != 0x0101)) {
		fprintf(stderr, "Unknown DFU version: %04x\n", suffix->bcdDFU);
		return 1;
	}

	// Suffix bytes are reversed
	if(!((suffix->ucDfuSig[0]==0x55) &&
	     (suffix->ucDfuSig[1]==0x46) &&
	     (suffix->ucDfuSig[2]==0x44))) {
		fprintf(stderr, "DFU Signature mismatch: not a DFU file\n");
		return 1;
	}

	fseek(signedfile, 0, SEEK_SET);
	data = malloc(data_length);
	if(data == NULL) {
		fprintf(stderr, "Cannot allocate buffer for CRC check\n");
		return 1;
	}

	if (fread(data, 1, data_length, signedfile) != data_length) {
		perror("DFU file");
		free(data);
		return 1;
	}
	crc = crc32(data, data_length);
	free(data);
	if(crc != suffix->dwCRC) {
		fprintf(stderr, "CRC mismatch: calculated: 0x%x, found: 0x%x\n", crc, suffix->dwCRC);
		return 1;
	}
	return 0;
}

/* Add a DFU suffix to infile and write to outfile
 * suffix must already contain VID/PID
 */
int sign(FILE* infile, FILE* outfile, uint16_t idVendor, uint16_t idProduct) {
	DFU_suffix* suffix;
	uint32_t data_length, buffer_length;
	uint8_t* buffer;

	long file_length;

	if (fseek(infile, 0, SEEK_END) != 0 || (file_length = ftell(infile)) < 0) {
		perror("input file");
		return 1;
	}
	data_length = file_length;
	buffer_length = data_length + sizeof(DFU_suffix); // Add suffix
	buffer = malloc(buffer_length);
	if(buffer == NULL) {
		fprintf(stderr, "Cannot allocate buffer to calculate CRC\n");
		return 1;
	}

	fseek(infile, 0, SEEK_SET);
	if (fread(buffer, 1, data_length, infile) != data_length) {
		perror("input file");
		free(buffer);
		return 1;
	}

	suffix = (DFU_suffix *) (buffer + data_length);
	suffix->idVendor    = idVendor;
	suffix->idProduct   = idProduct;
	suffix->bcdDevice   = 0;
	suffix->bcdDFU      = 0x0100;
	suffix->ucDfuSig[0] = 0x55;
	suffix->ucDfuSig[1] = 0x46;
	suffix->ucDfuSig[2] = 0x44;
	suffix->bLength     = 16;
	suffix->dwCRC = crc32(buffer, data_length + 12);

	if (fwrite(buffer, 1, buffer_length, outfile) != buffer_length) {
		perror("output file");
		free(buffer);
		return 1;
	}
	free(buffer);
	return 0;
}

/*
 * USB helper function - find devices
 */
/* physical location of a USB device; stays the same when the Ubertooth
 * re-enumerates in DFU mode */
typedef struct {
	uint8_t bus;
	int nports;
	uint8_t ports[8];
} usb_location;

static void get_location(libusb_device* dev, usb_location* loc)
{
	loc->bus = libusb_get_bus_number(dev);
	loc->nports = libusb_get_port_numbers(dev, loc->ports, sizeof(loc->ports));
	if (loc->nports < 0)
		loc->nports = 0;
}

static int same_location(const usb_location* a, const usb_location* b)
{
	return a->bus == b->bus && a->nports == b->nports &&
	       memcmp(a->ports, b->ports, a->nports) == 0;
}

/*
 * Find an Ubertooth in DFU mode. If loc is given, only a device at that
 * physical location matches (the one we just switched to DFU mode).
 * Otherwise exactly one DFU-mode device must be present.
 * libusb must already be initialized.
 */
static struct libusb_device_handle* find_ubertooth_dfu_device(const usb_location* loc,
                                                              int quiet) {
	struct libusb_device **usb_list = NULL;
	struct libusb_device_handle *devh = NULL;
	struct libusb_device_descriptor desc;
	libusb_device* found = NULL;
	ssize_t usb_devs, i;
	int matches = 0, ret;

	usb_devs = libusb_get_device_list(NULL, &usb_list);
	if (usb_devs < 0) {
		show_libusb_error((int)usb_devs);
		return NULL;
	}
	for(i = 0 ; i < usb_devs ; ++i) {
		if (libusb_get_device_descriptor(usb_list[i], &desc) < 0) {
			fprintf(stderr, "couldn't get usb descriptor for dev #%zd!\n", i);
			continue;
		}
		if ((desc.idVendor == TC13_VENDORID && desc.idProduct == TC13_PRODUCTID) ||
		    (desc.idVendor == U1_DFU_VENDORID && desc.idProduct == U1_DFU_PRODUCTID))
		{
			if (loc) {
				usb_location l;
				get_location(usb_list[i], &l);
				if (!same_location(loc, &l))
					continue;
			}
			found = usb_list[i];
			matches++;
		}
	}

	if (matches > 1) {
		if (!quiet)
			fprintf(stderr, "Multiple Uberteeth in DFU mode found; unplug all but one\n");
	} else if (found) {
		ret = libusb_open(found, &devh);
		if (ret) {
			show_libusb_error(ret);
			devh = NULL;
		}
	}

	libusb_free_device_list(usb_list, 1);
	return devh;
}

void stop_device(struct libusb_device_handle *devh)
{
	if (devh != NULL) {
		libusb_release_interface(devh, 0);
		libusb_close(devh);
	}
}

/*
 * DFU functions
 */
#define DFU_IN LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE
#define DFU_OUT LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE

int dfu_get_state(libusb_device_handle* devh) {
	int rv;
	uint8_t state;
	rv = libusb_control_transfer(devh, DFU_IN, REQ_GETSTATE, 0, 0, &state, 1, 1000);
	if (rv < 0) {
		if (rv == LIBUSB_ERROR_PIPE)
			fprintf(stderr, "control message unsupported\n");
		else
			show_libusb_error(rv);
		return rv;
	}
	return state;
}

int dfu_clear_status(libusb_device_handle* devh) {
	int rv;
	rv = libusb_control_transfer(devh, DFU_OUT, REQ_CLRSTATUS, 0, 0, NULL, 0, 1000);
	if (rv < 0) {
		if (rv == LIBUSB_ERROR_PIPE)
			fprintf(stderr, "control message unsupported\n");
		else
			show_libusb_error(rv);
		return rv;
	}
	return 0;
}

int dfu_abort(libusb_device_handle* devh) {
	int rv;
	rv = libusb_control_transfer(devh, DFU_OUT, REQ_ABORT, 0, 0, NULL, 0, 1000);
	if (rv < 0) {
		if (rv == LIBUSB_ERROR_PIPE)
			fprintf(stderr, "control message unsupported\n");
		else
			show_libusb_error(rv);
		return rv;
	}
	return 0;
}

int enter_dfu_mode(libusb_device_handle* devh) {
	int state;
	int rv;
	int tries;
	for (tries = 0; ; tries++) {
		state = dfu_get_state(devh);
		if (state < 0)
			return state;
		if(state == STATE_DFU_IDLE)
			break;
		if (tries >= 20) {
			fprintf(stderr, "Device stuck in DFU state %d\n", state);
			return -1;
		}
		switch(state) {
			case STATE_DFU_DNLOAD_SYNC:
			case STATE_DFU_DNLOAD_IDLE:
			case STATE_DFU_MANIFEST_SYNC:
			case STATE_DFU_UPLOAD_IDLE:
			case STATE_DFU_MANIFEST:
				rv = dfu_abort(devh);
				if(rv < 0) {
					fprintf(stderr, "Unable to abort transaction from state:%d\n", state);
					return rv;
				}
				break;
			case STATE_APP_DETACH:
			case STATE_DFU_DNBUSY:
			case STATE_DFU_MANIFEST_WAIT_RESET:
				sleep(1);
				break;
			case STATE_APP_IDLE:
				// We don't support this state in the application
				fprintf(stderr, "Device is not in DFU mode (appIDLE)\n");
				return -1;
			case STATE_DFU_ERROR:
				rv = dfu_clear_status(devh);
				if (rv < 0)
					return rv;
				break;
			default:
				fprintf(stderr, "Unknown DFU state %d\n", state);
				return -1;
		}
	}
	return 0;
}

int detach(libusb_device_handle* devh) {
	int state, rv;
	rv = enter_dfu_mode(devh);
	if(rv < 0) {
		fprintf(stderr, "Detach failed: could not enter DFU mode\n");
		return rv;
	}
	state = dfu_get_state(devh);
	if(state == STATE_DFU_IDLE) {
		rv = libusb_control_transfer(devh, DFU_OUT, REQ_DETACH, 0, 0, NULL, 0, 1000);
		if (rv != LIBUSB_SUCCESS) {
			if (rv == LIBUSB_ERROR_PIPE) {
				fprintf(stderr, "control message unsupported\n");
			} else
				show_libusb_error(rv);
			return rv;
		} else
			printf("Detached\n");
	} else {
		fprintf(stderr, "In unexpected state: %d", state);
		return 1;
	}
	return 0;
}

int upload(libusb_device_handle* devh, FILE* upfile) {
	int address, length, block, rv;
	uint8_t buffer[BLOCK_SIZE];
	address = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
	length = (256 * 1024) - address;
	block = address / BLOCK_SIZE;

	if ((address & (BLOCK_SIZE - 1)) != 0) {
		fprintf(stderr, "Upload failed: must start at block boundary\n");
		return -1;
	}

	rv = enter_dfu_mode(devh);
	if(rv < 0) {
		fprintf(stderr, "Upload failed: could not enter DFU mode\n");
		return rv;
	}

	while(length > 0) {
		rv = libusb_control_transfer(devh, DFU_IN, REQ_UPLOAD, block, 0,
		                             buffer, BLOCK_SIZE, 1000);
		if (rv < 0) {
			if (rv == LIBUSB_ERROR_PIPE)
				fprintf(stderr, "control message unsupported\n");
			else {
				show_libusb_error(rv);
				return rv;
			}
		}
		fprintf(stdout, ".");
		if(rv == BLOCK_SIZE) {
			if (fwrite(buffer, 1, BLOCK_SIZE, upfile) != BLOCK_SIZE) {
				fprintf(stdout, "\n");
				perror("Upload failed: writing output file");
				return -1;
			}
		} else {
			fprintf(stdout, "\n");
			fprintf(stderr, "Upload failed: did not read full block\n");
			return -1;
		}
		block++;
		length -= rv;
	}
	fprintf(stdout, "\n");
	return 0;
}

/*
 * DFU_GETSTATUS. Returns <0 on a USB error, otherwise fills in bStatus,
 * bState and bwPollTimeout.
 */
int dfu_get_status(libusb_device_handle* devh, uint8_t* status, uint8_t* state,
                   unsigned* poll_ms) {
	uint8_t buffer[6];
	int rv;
	rv = libusb_control_transfer(devh, DFU_IN, REQ_GETSTATUS, 0, 0, buffer, 6, 1000);
	if (rv < 0) {
		if (rv == LIBUSB_ERROR_PIPE)
			fprintf(stderr, "DFU_GETSTATUS stalled\n");
		else
			show_libusb_error(rv);
		return rv;
	}
	if (rv < 6) {
		fprintf(stderr, "short DFU_GETSTATUS response (%d bytes)\n", rv);
		return LIBUSB_ERROR_IO;
	}
	*status = buffer[0];
	*poll_ms = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16);
	*state = buffer[4];
	return 0;
}

/* wait until the device has finished processing a download block */
static int dfu_wait_block(libusb_device_handle* devh, int block) {
	uint8_t status, state;
	unsigned poll_ms;
	int rv, tries;

	for (tries = 0; tries < 100; tries++) {
		rv = dfu_get_status(devh, &status, &state, &poll_ms);
		if (rv < 0)
			return rv;
		if (status != STATUS_OK || state == STATE_DFU_ERROR) {
			fprintf(stderr, "\nDevice reported error 0x%x (state %u) writing block %d\n",
			        status, state, block);
			return -1;
		}
		if (state == STATE_DFU_DNLOAD_IDLE || state == STATE_DFU_IDLE)
			return 0;
		if (state != STATE_DFU_DNBUSY && state != STATE_DFU_DNLOAD_SYNC) {
			fprintf(stderr, "\nUnexpected DFU state %u after block %d\n", state, block);
			return -1;
		}
		usleep((poll_ms ? poll_ms : 1) * 1000);
	}
	fprintf(stderr, "\nTimed out waiting for block %d to be written\n", block);
	return -1;
}

/* firmware_length: number of bytes to flash (the file minus its DFU suffix) */
int download(libusb_device_handle* devh, FILE* downfile, long firmware_length) {
	int address, length, block, rv, block_count=1;
	uint8_t buffer[BLOCK_SIZE];
	long remaining = firmware_length;
	address = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
	block = address / BLOCK_SIZE;
	if (fseek(downfile, 0, SEEK_SET) != 0) {
		perror("DFU file");
		return -1;
	}

	if ((address & (SECTOR_SIZE - 1)) != 0) {
		fprintf(stderr, "Download failed: must start at sector boundary\n");
		return -1;
	}

	rv = enter_dfu_mode(devh);
	if(rv < 0) {
		fprintf(stderr, "Download failed: could not enter DFU mode\n");
		return rv;
	}
	while (remaining > 0) {
		size_t want = remaining < BLOCK_SIZE ? (size_t)remaining : BLOCK_SIZE;
		length = fread(buffer, 1, want, downfile);
		if ((size_t)length != want) {
			fprintf(stderr, "\nDownload failed: could not read firmware file\n");
			return -1;
		}
		remaining -= length;
		for(;length<BLOCK_SIZE;length++)
			buffer[length] = 0xFF;
		rv = libusb_control_transfer(devh, DFU_OUT, REQ_DNLOAD, block, 0,
		                             buffer, BLOCK_SIZE, 1000);
		if (rv != BLOCK_SIZE) {
			/* the bootloader stalls DNLOAD when the flash write fails */
			fprintf(stderr, "\nDownload failed at block %d: ", block);
			if (rv == LIBUSB_ERROR_PIPE)
				fprintf(stderr, "device rejected the block\n");
			else if (rv < 0)
				show_libusb_error(rv);
			else
				fprintf(stderr, "short write (%d bytes)\n", rv);
			return -1;
		}
		rv = dfu_wait_block(devh, block);
		if (rv < 0)
			return rv;
		fprintf(stdout, ".");
		if(block_count % 40)
			fflush(stdout);
		else
			fprintf(stdout, "\n");
		block++;
		block_count++;
	}
	fprintf(stdout, "\n");
	return 0;
}

static int get_outfile(char *infile, char **outfile) {
	char *suffix = strrchr(infile, '.');
	if (suffix != NULL && strcmp(suffix, ".bin") == 0) {
		*outfile = strdup(infile);
		if (*outfile == NULL)
			return 0;
		suffix = strrchr(*outfile, '.');
		strcpy(suffix, ".dfu");
		return 1;
	} else {
		return 0;
	}
}

static void usage()
{
	printf("ubertooth-dfu - Ubertooth firmware update tool\n");
	printf("\n");
	printf("To update firmware, run:\n");
	printf("\tubertooth-dfu -d bluetooth_rxtx.dfu -r\n");
	printf("\n");
	printf("Usage:\n");
	printf("\t-u <filename> upload - read firmware from device\n");
	printf("\t-d <filename> download - write DFU file to device\n");
	printf("\t-r reset Ubertooth after other operations complete\n");
	printf("\n");
	printf("Miscellaneous:\n");
	printf("\t-s <filename> add DFU suffix to binary firmware file\n");
	printf("\t-U <0-7> set ubertooth device to use\n");
}

#define FUNC_DOWNLOAD (1<<0)
#define FUNC_UPLOAD   (1<<1)
#define FUNC_SIGN     (1<<2)
#define FUNC_RESET    (1<<3)

int main(int argc, char **argv) {
	FILE* downfile = NULL;
	FILE* upfile = NULL;
	FILE* infile = NULL;
	FILE* outfile = NULL;
	char* outfile_name;
	libusb_device_handle* devh = NULL;
	uint8_t functions = 0;
	int opt, ubertooth_device = -1;
	int r;
	int ret = 0;
	DFU_suffix suffix;
	ubertooth_t* ut = NULL;

	while ((opt=getopt(argc,argv,"hd:u:s:rU:")) != EOF) {
		switch(opt) {
		case 'd':
			downfile = fopen(optarg, "r+b");
			if (downfile == NULL) {
				perror(optarg);
				return 1;
			}
			functions |= FUNC_DOWNLOAD;
			break;
		case 'u':
			upfile = fopen(optarg, "w+b");
			if (upfile == NULL) {
				perror(optarg);
				return 1;
			}
			functions |= FUNC_UPLOAD;
			break;
		case 's':
			infile = fopen(optarg, "r+b");
			if (infile == NULL) {
				perror(optarg);
				return 1;
			}
			r = get_outfile(optarg, &outfile_name);
			if (r == 0) {
				printf("Error: -s requires a .bin\n");
				return 1;
			}
			outfile = fopen(outfile_name, "w+b");
			if (outfile == NULL) {
				perror(optarg);
				return 1;
			}
			free(outfile_name);
			functions |= FUNC_SIGN;
			break;
		case 'r':
			functions |= FUNC_RESET;
			break;
		case 'U':
			ubertooth_device = atoi(optarg);
			break;
		default:
		case 'h':
			usage();
			return 1;
		}
	}

	if(functions & FUNC_SIGN) {
		r = sign(infile, outfile, U1_DFU_VENDORID, U1_DFU_PRODUCTID);
		fclose(infile);
		if (fclose(outfile) != 0 || r != 0) {
			fprintf(stderr, "Signing failed\n");
			return 1;
		}
	}

	if(functions & FUNC_DOWNLOAD) {
		/* verify the file before touching the device */
		r = check_suffix(downfile, &suffix);
		if(r) {
			fprintf(stderr, "Signature check failed, firmware will not be updated\n");
			return 1;
		}
	}

	if(functions & (FUNC_UPLOAD|FUNC_DOWNLOAD|FUNC_RESET)) {
		int rv, count= 0;

		rv = libusb_init(NULL);
		if (rv < 0) {
			show_libusb_error(rv);
			return 1;
		}

		if (ubertooth_device < 0) {
			/* any single device already in DFU mode */
			devh = find_ubertooth_dfu_device(NULL, 1);
		}
		if(devh == NULL) {
			usb_location loc;

			// Find Ubertooth and switch it to DFU mode
			ut = ubertooth_start(ubertooth_device);
			if(ut == NULL) {
				fprintf(stderr, "Unable to find Ubertooth\n");
				libusb_exit(NULL);
				return 1;
			}
			/* remember where it is plugged in so we find the same
			 * device after it re-enumerates in DFU mode */
			get_location(libusb_get_device(ut->devh), &loc);
			cmd_flash(ut->devh);
			/* close without sending STOP: the device is rebooting */
			libusb_release_interface(ut->devh, 0);
			libusb_close(ut->devh);
			ut->devh = NULL;
			ubertooth_free(ut);
			ut = NULL;

			fprintf(stdout, "Switching to DFU mode...\n");
			while(((devh = find_ubertooth_dfu_device(&loc, 0)) == NULL) && (count++) < 5)
				sleep(1);
			if(devh==NULL) {
				fprintf(stderr, "Unable to find Ubertooth in DFU mode\n");
				libusb_exit(NULL);
				return 1;
			}
		}
		rv = libusb_claim_interface(devh, 0);
		if (rv < 0) {
			fprintf(stderr, "usb_claim_interface error %d\n", rv);
			fprintf(stderr, "Correct permissions to access Ubertooth?\n");
			stop_device(devh);
			libusb_exit(NULL);
			return 1;
		}
	}

	if(functions & FUNC_UPLOAD) {
		int rv;
		rv = upload(devh, upfile);
		if (fclose(upfile) != 0 && rv == 0) {
			perror("upload file");
			rv = -1;
		}
		if(rv) {
			fprintf(stderr, "Firmware upload failed\n");
			ret = 1;
			goto out;
		}
	}

	if(functions & FUNC_DOWNLOAD) {
		int rv;
		long firmware_length;

		/* the DFU suffix is not part of the image */
		fseek(downfile, 0, SEEK_END);
		firmware_length = ftell(downfile) - suffix.bLength;
		rv = download(devh, downfile, firmware_length);
		fclose(downfile);
		if(rv) {
			fprintf(stderr, "Firmware update failed; the device stays in DFU mode.\n");
			fprintf(stderr, "Run ubertooth-dfu again to retry.\n");
			ret = 1;
			goto out;
		}
	}

	if(functions & FUNC_RESET) {
		if (detach(devh) != 0)
			ret = 1;
	}

out:
	if(functions & (FUNC_UPLOAD|FUNC_DOWNLOAD|FUNC_RESET)) {
		stop_device(devh);
		libusb_exit(NULL);
	}
	return ret;
}

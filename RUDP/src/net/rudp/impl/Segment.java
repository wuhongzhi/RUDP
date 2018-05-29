/*
 * Simple Reliable UDP (rudp)
 * Copyright (c) 2009, Adrian Granados (agranados@ihmc.us)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
package net.rudp.impl;

import java.nio.ByteBuffer;

public abstract class Segment {
	public static final byte SYN_FLAG = (byte) 0x80;
	public static final byte ACK_FLAG = (byte) 0x40;
	public static final byte EAK_FLAG = (byte) 0x20;
	public static final byte RST_FLAG = (byte) 0x10;
	public static final byte NUL_FLAG = (byte) 0x08;
	public static final byte CHK_FLAG = (byte) 0x04;
	public static final byte FIN_FLAG = (byte) 0x02;
	public static final byte CLZ_FLAG = (byte) 0x01;
	public static final int RUDP_HEADER_LEN = 6;
	public static final int RUDP_VERSION = 1;

	public static Segment parse(byte[] bytes) {
		return Segment.parse(bytes, 0, bytes.length);
	}

	public static Segment parse(byte[] bytes, int off, int len) {
		Segment segment = null;
		if (len < RUDP_HEADER_LEN) {
			throw new IllegalArgumentException("Invalid segment");
		}
		int flags = bytes[off];
		if ((flags & SYN_FLAG) != 0) {
			segment = new SYNSegment();
		} else if ((flags & NUL_FLAG) != 0) {
			segment = new NULSegment();
		} else if ((flags & EAK_FLAG) != 0) {
			segment = new EAKSegment();
		} else if ((flags & RST_FLAG) != 0) {
			segment = new RSTSegment();
		} else if ((flags & CLZ_FLAG) != 0) {
			segment = new CLZSegment();
		} else if ((flags & FIN_FLAG) != 0) {
			segment = new FINSegment();
		} else if ((flags
				& ACK_FLAG) != 0) { /*
									 * always process ACKs or Data segments last
									 */
			if (len == RUDP_HEADER_LEN) {
				segment = new ACKSegment();
			} else {
				segment = new DATSegment();
			}
		}
		if (segment == null) {
			throw new IllegalArgumentException("Invalid segment");
		}
		segment.parseBytes(bytes, off, len);
		return segment;
	}

	private int _ackn; /* Acknowledgment number field */
	private int _flags; /* Control flags field */
	private int _hlen; /* Header length field */
	private int _nretx; /* Retransmission counter */
	private int _seqn; /* Sequence number field */

	protected Segment() {
		_nretx = 0;
		_ackn = -1;
	}

	public byte[] withChecksum() {
		ByteBuffer buffer = ByteBuffer.wrap(getBytes());
		buffer.putShort(_hlen - 2, (short)0);
		long checksum = checksum(buffer.array(), 0, buffer.limit());
		buffer.putShort(_hlen - 2, (short)checksum);
		return buffer.array();
	}

	long checksum(byte[] buf, int off, int length) {
		int count = (_flags & CHK_FLAG) != 0 ? length : _hlen;
		int i = off;
		long sum = 0;
		while (count > 0) {
			sum += (buf[i++] & 0xFF) << 8;
			if ((--count) == 0) break;
			sum += (buf[i++] & 0xFF);
			--count;
		}
		return (~((sum & 0xFFFF) + (sum >> 16))) & 0xFFFF;
	}

	public int flags() {
		return _flags;
	}

	public int getAck() {
		if ((_flags & ACK_FLAG) == ACK_FLAG) {
			return _ackn;
		}
		return -1;
	}

	public byte[] getBytes() {
		byte[] buffer = new byte[length()];
		buffer[0] = (byte) (_flags & 0xFF);
		buffer[1] = (byte) (_hlen & 0xFF);
		buffer[2] = (byte) (_seqn & 0xFF);
		buffer[3] = (byte) (_ackn & 0xFF);
		return buffer;
	}

	public int getRetxCounter() {
		return _nretx;
	}

	/*
	 * RUDP Header
	 *
	 * 0 1 2 3 4 5 6 7 8             15 
	 * +-+-+-+-+-+-+-+-+---------------+ 
	 * |S|A|E|R|N|C|F|C| Header        | 
	 * |Y|C|A|S|U|H|I|L| Length        | 
	 * |N|K|K|T|L|K|N|Z|               |
	 * +-+-+-+-+-+-+-+-+---------------+ 
	 * | Sequence # + Ack Number       |
	 * +---------------+---------------+ 
	 * | Checksum                      |
	 * +---------------+---------------+
	 */
	protected void init(int flags, int seqn, int len) {
		_flags = flags;
		_seqn = seqn;
		_hlen = len;
	}

	public int length() {
		return _hlen;
	}

	protected void parseBytes(byte[] buffer, int off, int len) {
		_flags = (buffer[off] & 0xFF);
		_hlen = (buffer[off + 1] & 0xFF);
		_seqn = (buffer[off + 2] & 0xFF);
		_ackn = (buffer[off + 3] & 0xFF);
		if (checksum(buffer, off, len) != 0) {
			throw new IllegalArgumentException("Invalid segment");
		}
	}

	public int seq() {
		return _seqn;
	}

	public void setAck(int ackn) {
		_flags = _flags | ACK_FLAG;
		_ackn = ackn;
	}

	public void setRetxCounter(int n) {
		_nretx = n;
	}

	public String toString() {
		return type() + " [" + " SEQ = " + seq() 
				+ ", ACK = " + ((getAck() >= 0) ? "" + getAck() : "N/A") + ", LEN = "
				+ length() + " ]";
	}

	public abstract String type();
}

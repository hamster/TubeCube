/*
 *  Project     Segmented LED Display - ASCII Library
 *  @author     David Madison
 *  @link       github.com/dmadison/Segmented-LED-Display-ASCII
 *  @license    MIT - Copyright (c) 2017 David Madison
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
 */


#ifndef CHARACTERS_H_
#define CHARACTERS_H_

const uint8_t TubeChar[][3] = {
 { 0b00000000, 0b00000000, 0b00000000 }, //  (space)
 { 0b00000000, 0b00000000, 0b01011000 }, //  !
 { 0b00001000, 0b00000000, 0b01000000 }, //  "
 { 0b00001010, 0b00010101, 0b01001100 }, //  #
 { 0b00001011, 0b10010101, 0b10001100 }, //  $
 { 0b00001011, 0b10001111, 0b00001100 }, //  %
 { 0b00001111, 0b00110000, 0b00000110 }, //  &
 { 0b00001000, 0b00000000, 0b00000000 }, //  '
 { 0b00000000, 0b00000010, 0b00000010 }, //  (
 { 0b00000100, 0b00001000, 0b00000000 }, //  )
 { 0b00001110, 0b00001111, 0b00000010 }, //  *
 { 0b00001010, 0b00000101, 0b00000000 }, //  +
 { 0b00000000, 0b01000000, 0b00000000 }, //  ,
 { 0b00000010, 0b00000001, 0b00000000 }, //  -
 { 0b00000000, 0b00000000, 0b00010000 }, //  .
 { 0b00000000, 0b00001010, 0b00000000 }, //  /
 { 0b00000001, 0b10111010, 0b11001100 }, //  0
 { 0b00000000, 0b00000010, 0b01001000 }, //  1
 { 0b00000011, 0b00110001, 0b11000100 }, //  2
 { 0b00000001, 0b00010001, 0b11001100 }, //  3
 { 0b00000010, 0b10000001, 0b01001000 }, //  4
 { 0b00000011, 0b10010000, 0b10000110 }, //  5
 { 0b00000011, 0b10110001, 0b10001100 }, //  6
 { 0b00000001, 0b00000000, 0b11001000 }, //  7
 { 0b00000011, 0b10110001, 0b11001100 }, //  8
 { 0b00000011, 0b10010001, 0b11001100 }, //  9
 { 0b00001000, 0b00000100, 0b00000000 }, //  :
 { 0b00001000, 0b00001000, 0b00000000 }, //  ;
 { 0b00000010, 0b00000010, 0b00000010 }, //  <
 { 0b00000010, 0b00010001, 0b00000100 }, //  =
 { 0b00000100, 0b00001001, 0b00000000 }, //  >
 { 0b00000001, 0b00000101, 0b11010000 }, //  ?
 { 0b00001001, 0b10110001, 0b11000100 }, //  @
 { 0b00000011, 0b10100001, 0b11001000 }, //  A
 { 0b00001001, 0b00010101, 0b11001100 }, //  B
 { 0b00000001, 0b10110000, 0b10000100 }, //  C
 { 0b00001001, 0b00010100, 0b11001100 }, //  D
 { 0b00000011, 0b10110000, 0b10000100 }, //  E
 { 0b00000011, 0b10100000, 0b10000000 }, //  F
 { 0b00000001, 0b10110001, 0b10001100 }, //  G
 { 0b00000010, 0b10100001, 0b01001000 }, //  H
 { 0b00001001, 0b00010100, 0b10000100 }, //  I
 { 0b00000000, 0b00110000, 0b01001100 }, //  J
 { 0b00000010, 0b10100010, 0b00000010 }, //  K
 { 0b00000000, 0b10110000, 0b00000100 }, //  L
 { 0b00000100, 0b10100010, 0b01001000 }, //  M
 { 0b00000100, 0b10100000, 0b01001010 }, //  N
 { 0b00000001, 0b10110000, 0b11001100 }, //  O
 { 0b00000011, 0b10100001, 0b11000000 }, //  P
 { 0b00000001, 0b10110000, 0b11001110 }, //  Q
 { 0b00000011, 0b10100001, 0b11000010 }, //  R
 { 0b00000011, 0b10010001, 0b10001100 }, //  S
 { 0b00001001, 0b00000100, 0b10000000 }, //  T
 { 0b00000000, 0b10110000, 0b01001100 }, //  U
 { 0b00000000, 0b10101010, 0b00000000 }, //  V
 { 0b00000000, 0b10101000, 0b01001010 }, //  W
 { 0b00000100, 0b00001010, 0b00000010 }, //  X
 { 0b00000010, 0b10010001, 0b01001100 }, //  Y
 { 0b00000001, 0b00011010, 0b10000100 }, //  Z
 { 0b00001000, 0b00000100, 0b10000100 }, //  [
 { 0b00000100, 0b00000000, 0b00000010 }, //  Backslash
 { 0b00001001, 0b00010100, 0b00000000 }, //  ]
 { 0b00000000, 0b00001000, 0b00000010 }, //  ^
 { 0b00000000, 0b00010000, 0b00000100 }, //  _
 { 0b00000100, 0b00000000, 0b00000000 }, //  `
 { 0b00000010, 0b00110100, 0b00000100 }, //  a
 { 0b00000010, 0b10110100, 0b00000000 }, //  b
 { 0b00000010, 0b00110000, 0b00000000 }, //  c
 { 0b00000000, 0b00000101, 0b01001100 }, //  d
 { 0b00000010, 0b00111000, 0b00000000 }, //  e
 { 0b00001010, 0b00000101, 0b10000000 }, //  f
 { 0b00001011, 0b10010100, 0b00000000 }, //  g
 { 0b00000010, 0b10100100, 0b00000000 }, //  h
 { 0b00000000, 0b00000100, 0b00000000 }, //  i
 { 0b00001000, 0b00110100, 0b00000000 }, //  j
 { 0b00001000, 0b00000110, 0b00000010 }, //  k
 { 0b00000000, 0b10100000, 0b00000000 }, //  l
 { 0b00000010, 0b00100101, 0b00001000 }, //  m
 { 0b00000010, 0b00100100, 0b00000000 }, //  n
 { 0b00000010, 0b00110100, 0b00000000 }, //  o
 { 0b00001011, 0b10100000, 0b00000000 }, //  p
 { 0b00001011, 0b10000100, 0b00000000 }, //  q
 { 0b00000010, 0b00100000, 0b00000000 }, //  r
 { 0b00000011, 0b10010100, 0b00000000 }, //  s
 { 0b00000010, 0b10110000, 0b00000000 }, //  t
 { 0b00000000, 0b00110100, 0b00000000 }, //  u
 { 0b00000000, 0b00101000, 0b00000000 }, //  v
 { 0b00000000, 0b00101000, 0b00001010 }, //  w
 { 0b00000100, 0b00001010, 0b00000010 }, //  x
 { 0b00001000, 0b00000001, 0b01001100 }, //  y
 { 0b00000010, 0b00011000, 0b00000000 }, //  z
 { 0b00001010, 0b00000100, 0b10000100 }, //  {
 { 0b00001000, 0b00000100, 0b00000000 }, //  |
 { 0b00001001, 0b00010101, 0b00000000 }, //  }
 { 0b00000010, 0b00001011, 0b00000000 } //  ~
};

const uint8_t TubeSpin[8][3] = {
	{ 0b00000001, 0b00000000, 0b00000000 }, // A1 
	{ 0b00000000, 0b00000000, 0b10000000 }, // A2
	{ 0b00000000, 0b00000000, 0b01000000 }, // B
	{ 0b00000000, 0b00000000, 0b00001000 }, // C
	{ 0b00000000, 0b00000000, 0b00000100 }, // D2
	{ 0b00000000, 0b00010000, 0b00000000 }, // D1
	{ 0b00000000, 0b00100000, 0b00000000 }, // E
	{ 0b00000000, 0b10000000, 0b00000000 } // F
};

#endif /* CHARACTERS_H_ */
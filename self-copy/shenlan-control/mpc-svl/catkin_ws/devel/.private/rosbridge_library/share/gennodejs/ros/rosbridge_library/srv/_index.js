
"use strict";

let TestEmpty = require('./TestEmpty.js')
let TestMultipleResponseFields = require('./TestMultipleResponseFields.js')
let TestRequestAndResponse = require('./TestRequestAndResponse.js')
let AddTwoInts = require('./AddTwoInts.js')
let SendBytes = require('./SendBytes.js')
let TestMultipleRequestFields = require('./TestMultipleRequestFields.js')
let TestNestedService = require('./TestNestedService.js')
let TestArrayRequest = require('./TestArrayRequest.js')
let TestResponseOnly = require('./TestResponseOnly.js')
let TestRequestOnly = require('./TestRequestOnly.js')

module.exports = {
  TestEmpty: TestEmpty,
  TestMultipleResponseFields: TestMultipleResponseFields,
  TestRequestAndResponse: TestRequestAndResponse,
  AddTwoInts: AddTwoInts,
  SendBytes: SendBytes,
  TestMultipleRequestFields: TestMultipleRequestFields,
  TestNestedService: TestNestedService,
  TestArrayRequest: TestArrayRequest,
  TestResponseOnly: TestResponseOnly,
  TestRequestOnly: TestRequestOnly,
};

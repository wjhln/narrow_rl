уд
нГ
B
AssignVariableOp
resource
value"dtype"
dtypetypeИ
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
.
Identity

input"T
output"T"	
Ttype
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(И

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetypeИ
E
Relu
features"T
activations"T"
Ttype:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0И
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0И
?
Select
	condition

t"T
e"T
output"T"	
Ttype
H
ShardedFilename
basename	
shard

num_shards
filename
╛
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring И
@
StaticRegexFullMatch	
input

output
"
patternstring
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
Ц
VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 И"serve*2.6.22v2.6.1-9-gc2363d6d0258Ъ╓
Ь
deep_q_network_1/dense_3/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
▀А*0
shared_name!deep_q_network_1/dense_3/kernel
Х
3deep_q_network_1/dense_3/kernel/Read/ReadVariableOpReadVariableOpdeep_q_network_1/dense_3/kernel* 
_output_shapes
:
▀А*
dtype0
У
deep_q_network_1/dense_3/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:А*.
shared_namedeep_q_network_1/dense_3/bias
М
1deep_q_network_1/dense_3/bias/Read/ReadVariableOpReadVariableOpdeep_q_network_1/dense_3/bias*
_output_shapes	
:А*
dtype0
Ь
deep_q_network_1/dense_4/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
АА*0
shared_name!deep_q_network_1/dense_4/kernel
Х
3deep_q_network_1/dense_4/kernel/Read/ReadVariableOpReadVariableOpdeep_q_network_1/dense_4/kernel* 
_output_shapes
:
АА*
dtype0
У
deep_q_network_1/dense_4/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:А*.
shared_namedeep_q_network_1/dense_4/bias
М
1deep_q_network_1/dense_4/bias/Read/ReadVariableOpReadVariableOpdeep_q_network_1/dense_4/bias*
_output_shapes	
:А*
dtype0
Ы
deep_q_network_1/dense_5/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	А*0
shared_name!deep_q_network_1/dense_5/kernel
Ф
3deep_q_network_1/dense_5/kernel/Read/ReadVariableOpReadVariableOpdeep_q_network_1/dense_5/kernel*
_output_shapes
:	А*
dtype0
Т
deep_q_network_1/dense_5/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namedeep_q_network_1/dense_5/bias
Л
1deep_q_network_1/dense_5/bias/Read/ReadVariableOpReadVariableOpdeep_q_network_1/dense_5/bias*
_output_shapes
:*
dtype0

NoOpNoOp
П
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*╩
value└B╜ B╢
Ц
fc1
fc2
fc3
	optimizer
loss
	variables
regularization_losses
trainable_variables
		keras_api


signatures
h

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
h

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
h

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
 
 
*
0
1
2
3
4
5
 
*
0
1
2
3
4
5
н
	variables
metrics
non_trainable_variables
layer_regularization_losses
regularization_losses
 layer_metrics

!layers
trainable_variables
 
ZX
VARIABLE_VALUEdeep_q_network_1/dense_3/kernel%fc1/kernel/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEdeep_q_network_1/dense_3/bias#fc1/bias/.ATTRIBUTES/VARIABLE_VALUE

0
1

0
1
 
н
	variables
"metrics
#non_trainable_variables
$layer_regularization_losses
trainable_variables
%layer_metrics

&layers
regularization_losses
ZX
VARIABLE_VALUEdeep_q_network_1/dense_4/kernel%fc2/kernel/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEdeep_q_network_1/dense_4/bias#fc2/bias/.ATTRIBUTES/VARIABLE_VALUE

0
1

0
1
 
н
	variables
'metrics
(non_trainable_variables
)layer_regularization_losses
trainable_variables
*layer_metrics

+layers
regularization_losses
ZX
VARIABLE_VALUEdeep_q_network_1/dense_5/kernel%fc3/kernel/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEdeep_q_network_1/dense_5/bias#fc3/bias/.ATTRIBUTES/VARIABLE_VALUE

0
1

0
1
 
н
	variables
,metrics
-non_trainable_variables
.layer_regularization_losses
trainable_variables
/layer_metrics

0layers
regularization_losses
 
 
 
 

0
1
2
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
|
serving_default_input_1Placeholder*(
_output_shapes
:         ▀*
dtype0*
shape:         ▀
 
StatefulPartitionedCallStatefulPartitionedCallserving_default_input_1deep_q_network_1/dense_3/kerneldeep_q_network_1/dense_3/biasdeep_q_network_1/dense_4/kerneldeep_q_network_1/dense_4/biasdeep_q_network_1/dense_5/kerneldeep_q_network_1/dense_5/bias*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:         *(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8В *.
f)R'
%__inference_signature_wrapper_1904299
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
┌
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename3deep_q_network_1/dense_3/kernel/Read/ReadVariableOp1deep_q_network_1/dense_3/bias/Read/ReadVariableOp3deep_q_network_1/dense_4/kernel/Read/ReadVariableOp1deep_q_network_1/dense_4/bias/Read/ReadVariableOp3deep_q_network_1/dense_5/kernel/Read/ReadVariableOp1deep_q_network_1/dense_5/bias/Read/ReadVariableOpConst*
Tin

2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8В *)
f$R"
 __inference__traced_save_1904481
▌
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenamedeep_q_network_1/dense_3/kerneldeep_q_network_1/dense_3/biasdeep_q_network_1/dense_4/kerneldeep_q_network_1/dense_4/biasdeep_q_network_1/dense_5/kerneldeep_q_network_1/dense_5/bias*
Tin
	2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8В *,
f'R%
#__inference__traced_restore_1904509╤й
 
Ж
%__inference_signature_wrapper_1904299
input_1
unknown:
▀А
	unknown_0:	А
	unknown_1:
АА
	unknown_2:	А
	unknown_3:	А
	unknown_4:
identityИвStatefulPartitionedCallЗ
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:         *(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8В *+
f&R$
"__inference__wrapped_model_19041412
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:         2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:Q M
(
_output_shapes
:         ▀
!
_user_specified_name	input_1
°
Щ
)__inference_dense_3_layer_call_fn_1904401

inputs
unknown:
▀А
	unknown_0:	А
identityИвStatefulPartitionedCallї
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:         А*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8В *M
fHRF
D__inference_dense_3_layer_call_and_return_conditional_losses_19041592
StatefulPartitionedCall|
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:         А2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         ▀: : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:         ▀
 
_user_specified_nameinputs
к

Ў
D__inference_dense_5_layer_call_and_return_conditional_losses_1904431

inputs1
matmul_readvariableop_resource:	А-
biasadd_readvariableop_resource:
identityИвBiasAdd/ReadVariableOpвMatMul/ReadVariableOpО
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	А*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2
MatMulМ
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOpБ
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2	
BiasAddk
IdentityIdentityBiasAdd:output:0^NoOp*
T0*'
_output_shapes
:         2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         А: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:         А
 
_user_specified_nameinputs
О
°
D__inference_dense_4_layer_call_and_return_conditional_losses_1904412

inputs2
matmul_readvariableop_resource:
АА.
biasadd_readvariableop_resource:	А
identityИвBiasAdd/ReadVariableOpвMatMul/ReadVariableOpП
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
АА*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
MatMulН
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02
BiasAdd/ReadVariableOpВ
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2	
BiasAddY
ReluReluBiasAdd:output:0*
T0*(
_output_shapes
:         А2
Relun
IdentityIdentityRelu:activations:0^NoOp*
T0*(
_output_shapes
:         А2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         А: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:         А
 
_user_specified_nameinputs
▒	
С
2__inference_deep_q_network_1_layer_call_fn_1904381	
state
unknown:
▀А
	unknown_0:	А
	unknown_1:
АА
	unknown_2:	А
	unknown_3:	А
	unknown_4:
identityИвStatefulPartitionedCall░
StatefulPartitionedCallStatefulPartitionedCallstateunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:         *(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8В *V
fQRO
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_19041992
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:         2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
(
_output_shapes
:         ▀

_user_specified_namestate
¤
Ю
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904199	
state#
dense_3_1904160:
▀А
dense_3_1904162:	А#
dense_4_1904177:
АА
dense_4_1904179:	А"
dense_5_1904193:	А
dense_5_1904195:
identityИвdense_3/StatefulPartitionedCallвdense_4/StatefulPartitionedCallвdense_5/StatefulPartitionedCallТ
dense_3/StatefulPartitionedCallStatefulPartitionedCallstatedense_3_1904160dense_3_1904162*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:         А*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8В *M
fHRF
D__inference_dense_3_layer_call_and_return_conditional_losses_19041592!
dense_3/StatefulPartitionedCall╡
dense_4/StatefulPartitionedCallStatefulPartitionedCall(dense_3/StatefulPartitionedCall:output:0dense_4_1904177dense_4_1904179*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:         А*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8В *M
fHRF
D__inference_dense_4_layer_call_and_return_conditional_losses_19041762!
dense_4/StatefulPartitionedCall┤
dense_5/StatefulPartitionedCallStatefulPartitionedCall(dense_4/StatefulPartitionedCall:output:0dense_5_1904193dense_5_1904195*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:         *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8В *M
fHRF
D__inference_dense_5_layer_call_and_return_conditional_losses_19041922!
dense_5/StatefulPartitionedCallГ
IdentityIdentity(dense_5/StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:         2

Identity┤
NoOpNoOp ^dense_3/StatefulPartitionedCall ^dense_4/StatefulPartitionedCall ^dense_5/StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 2B
dense_3/StatefulPartitionedCalldense_3/StatefulPartitionedCall2B
dense_4/StatefulPartitionedCalldense_4/StatefulPartitionedCall2B
dense_5/StatefulPartitionedCalldense_5/StatefulPartitionedCall:O K
(
_output_shapes
:         ▀

_user_specified_namestate
К
К
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904347
input_1:
&dense_3_matmul_readvariableop_resource:
▀А6
'dense_3_biasadd_readvariableop_resource:	А:
&dense_4_matmul_readvariableop_resource:
АА6
'dense_4_biasadd_readvariableop_resource:	А9
&dense_5_matmul_readvariableop_resource:	А5
'dense_5_biasadd_readvariableop_resource:
identityИвdense_3/BiasAdd/ReadVariableOpвdense_3/MatMul/ReadVariableOpвdense_4/BiasAdd/ReadVariableOpвdense_4/MatMul/ReadVariableOpвdense_5/BiasAdd/ReadVariableOpвdense_5/MatMul/ReadVariableOpз
dense_3/MatMul/ReadVariableOpReadVariableOp&dense_3_matmul_readvariableop_resource* 
_output_shapes
:
▀А*
dtype02
dense_3/MatMul/ReadVariableOpН
dense_3/MatMulMatMulinput_1%dense_3/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_3/MatMulе
dense_3/BiasAdd/ReadVariableOpReadVariableOp'dense_3_biasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02 
dense_3/BiasAdd/ReadVariableOpв
dense_3/BiasAddBiasAdddense_3/MatMul:product:0&dense_3/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_3/BiasAddq
dense_3/ReluReludense_3/BiasAdd:output:0*
T0*(
_output_shapes
:         А2
dense_3/Reluз
dense_4/MatMul/ReadVariableOpReadVariableOp&dense_4_matmul_readvariableop_resource* 
_output_shapes
:
АА*
dtype02
dense_4/MatMul/ReadVariableOpа
dense_4/MatMulMatMuldense_3/Relu:activations:0%dense_4/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_4/MatMulе
dense_4/BiasAdd/ReadVariableOpReadVariableOp'dense_4_biasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02 
dense_4/BiasAdd/ReadVariableOpв
dense_4/BiasAddBiasAdddense_4/MatMul:product:0&dense_4/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_4/BiasAddq
dense_4/ReluReludense_4/BiasAdd:output:0*
T0*(
_output_shapes
:         А2
dense_4/Reluж
dense_5/MatMul/ReadVariableOpReadVariableOp&dense_5_matmul_readvariableop_resource*
_output_shapes
:	А*
dtype02
dense_5/MatMul/ReadVariableOpЯ
dense_5/MatMulMatMuldense_4/Relu:activations:0%dense_5/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2
dense_5/MatMulд
dense_5/BiasAdd/ReadVariableOpReadVariableOp'dense_5_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02 
dense_5/BiasAdd/ReadVariableOpб
dense_5/BiasAddBiasAdddense_5/MatMul:product:0&dense_5/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2
dense_5/BiasAdds
IdentityIdentitydense_5/BiasAdd:output:0^NoOp*
T0*'
_output_shapes
:         2

IdentityС
NoOpNoOp^dense_3/BiasAdd/ReadVariableOp^dense_3/MatMul/ReadVariableOp^dense_4/BiasAdd/ReadVariableOp^dense_4/MatMul/ReadVariableOp^dense_5/BiasAdd/ReadVariableOp^dense_5/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 2@
dense_3/BiasAdd/ReadVariableOpdense_3/BiasAdd/ReadVariableOp2>
dense_3/MatMul/ReadVariableOpdense_3/MatMul/ReadVariableOp2@
dense_4/BiasAdd/ReadVariableOpdense_4/BiasAdd/ReadVariableOp2>
dense_4/MatMul/ReadVariableOpdense_4/MatMul/ReadVariableOp2@
dense_5/BiasAdd/ReadVariableOpdense_5/BiasAdd/ReadVariableOp2>
dense_5/MatMul/ReadVariableOpdense_5/MatMul/ReadVariableOp:Q M
(
_output_shapes
:         ▀
!
_user_specified_name	input_1
╖	
У
2__inference_deep_q_network_1_layer_call_fn_1904364
input_1
unknown:
▀А
	unknown_0:	А
	unknown_1:
АА
	unknown_2:	А
	unknown_3:	А
	unknown_4:
identityИвStatefulPartitionedCall▓
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:         *(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8В *V
fQRO
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_19041992
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:         2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:Q M
(
_output_shapes
:         ▀
!
_user_specified_name	input_1
О
°
D__inference_dense_3_layer_call_and_return_conditional_losses_1904392

inputs2
matmul_readvariableop_resource:
▀А.
biasadd_readvariableop_resource:	А
identityИвBiasAdd/ReadVariableOpвMatMul/ReadVariableOpП
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
▀А*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
MatMulН
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02
BiasAdd/ReadVariableOpВ
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2	
BiasAddY
ReluReluBiasAdd:output:0*
T0*(
_output_shapes
:         А2
Relun
IdentityIdentityRelu:activations:0^NoOp*
T0*(
_output_shapes
:         А2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         ▀: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:         ▀
 
_user_specified_nameinputs
О
°
D__inference_dense_3_layer_call_and_return_conditional_losses_1904159

inputs2
matmul_readvariableop_resource:
▀А.
biasadd_readvariableop_resource:	А
identityИвBiasAdd/ReadVariableOpвMatMul/ReadVariableOpП
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
▀А*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
MatMulН
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02
BiasAdd/ReadVariableOpВ
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2	
BiasAddY
ReluReluBiasAdd:output:0*
T0*(
_output_shapes
:         А2
Relun
IdentityIdentityRelu:activations:0^NoOp*
T0*(
_output_shapes
:         А2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         ▀: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:         ▀
 
_user_specified_nameinputs
Щ
ч
 __inference__traced_save_1904481
file_prefix>
:savev2_deep_q_network_1_dense_3_kernel_read_readvariableop<
8savev2_deep_q_network_1_dense_3_bias_read_readvariableop>
:savev2_deep_q_network_1_dense_4_kernel_read_readvariableop<
8savev2_deep_q_network_1_dense_4_bias_read_readvariableop>
:savev2_deep_q_network_1_dense_5_kernel_read_readvariableop<
8savev2_deep_q_network_1_dense_5_bias_read_readvariableop
savev2_const

identity_1ИвMergeV2CheckpointsП
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*2
StaticRegexFullMatchc
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.part2
Constl
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part2	
Const_1Л
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: 2
Selectt

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: 2

StringJoinZ

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :2

num_shards
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 2
ShardedFilename/shardж
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: 2
ShardedFilenameЕ
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*Ч
valueНBКB%fc1/kernel/.ATTRIBUTES/VARIABLE_VALUEB#fc1/bias/.ATTRIBUTES/VARIABLE_VALUEB%fc2/kernel/.ATTRIBUTES/VARIABLE_VALUEB#fc2/bias/.ATTRIBUTES/VARIABLE_VALUEB%fc3/kernel/.ATTRIBUTES/VARIABLE_VALUEB#fc3/bias/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_namesЦ
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*!
valueBB B B B B B B 2
SaveV2/shape_and_slicesв
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0:savev2_deep_q_network_1_dense_3_kernel_read_readvariableop8savev2_deep_q_network_1_dense_3_bias_read_readvariableop:savev2_deep_q_network_1_dense_4_kernel_read_readvariableop8savev2_deep_q_network_1_dense_4_bias_read_readvariableop:savev2_deep_q_network_1_dense_5_kernel_read_readvariableop8savev2_deep_q_network_1_dense_5_bias_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *
dtypes
	22
SaveV2║
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixesб
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 2
MergeV2Checkpointsr
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: 2

Identity_

Identity_1IdentityIdentity:output:0^NoOp*
T0*
_output_shapes
: 2

Identity_1c
NoOpNoOp^MergeV2Checkpoints*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"!

identity_1Identity_1:output:0*N
_input_shapes=
;: :
▀А:А:
АА:А:	А:: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:&"
 
_output_shapes
:
▀А:!

_output_shapes	
:А:&"
 
_output_shapes
:
АА:!

_output_shapes	
:А:%!

_output_shapes
:	А: 

_output_shapes
::

_output_shapes
: 
к

Ў
D__inference_dense_5_layer_call_and_return_conditional_losses_1904192

inputs1
matmul_readvariableop_resource:	А-
biasadd_readvariableop_resource:
identityИвBiasAdd/ReadVariableOpвMatMul/ReadVariableOpО
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	А*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2
MatMulМ
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOpБ
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2	
BiasAddk
IdentityIdentityBiasAdd:output:0^NoOp*
T0*'
_output_shapes
:         2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         А: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:         А
 
_user_specified_nameinputs
Ї
Ч
)__inference_dense_5_layer_call_fn_1904440

inputs
unknown:	А
	unknown_0:
identityИвStatefulPartitionedCallЇ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:         *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8В *M
fHRF
D__inference_dense_5_layer_call_and_return_conditional_losses_19041922
StatefulPartitionedCall{
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*'
_output_shapes
:         2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         А: : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:         А
 
_user_specified_nameinputs
О
°
D__inference_dense_4_layer_call_and_return_conditional_losses_1904176

inputs2
matmul_readvariableop_resource:
АА.
biasadd_readvariableop_resource:	А
identityИвBiasAdd/ReadVariableOpвMatMul/ReadVariableOpП
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
АА*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
MatMulН
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02
BiasAdd/ReadVariableOpВ
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2	
BiasAddY
ReluReluBiasAdd:output:0*
T0*(
_output_shapes
:         А2
Relun
IdentityIdentityRelu:activations:0^NoOp*
T0*(
_output_shapes
:         А2

Identity
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         А: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:         А
 
_user_specified_nameinputs
Р 
▄
#__inference__traced_restore_1904509
file_prefixD
0assignvariableop_deep_q_network_1_dense_3_kernel:
▀А?
0assignvariableop_1_deep_q_network_1_dense_3_bias:	АF
2assignvariableop_2_deep_q_network_1_dense_4_kernel:
АА?
0assignvariableop_3_deep_q_network_1_dense_4_bias:	АE
2assignvariableop_4_deep_q_network_1_dense_5_kernel:	А>
0assignvariableop_5_deep_q_network_1_dense_5_bias:

identity_7ИвAssignVariableOpвAssignVariableOp_1вAssignVariableOp_2вAssignVariableOp_3вAssignVariableOp_4вAssignVariableOp_5Л
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*Ч
valueНBКB%fc1/kernel/.ATTRIBUTES/VARIABLE_VALUEB#fc1/bias/.ATTRIBUTES/VARIABLE_VALUEB%fc2/kernel/.ATTRIBUTES/VARIABLE_VALUEB#fc2/bias/.ATTRIBUTES/VARIABLE_VALUEB%fc3/kernel/.ATTRIBUTES/VARIABLE_VALUEB#fc3/bias/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_namesЬ
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*!
valueBB B B B B B B 2
RestoreV2/shape_and_slices╬
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*0
_output_shapes
:::::::*
dtypes
	22
	RestoreV2g
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:2

Identityп
AssignVariableOpAssignVariableOp0assignvariableop_deep_q_network_1_dense_3_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1╡
AssignVariableOp_1AssignVariableOp0assignvariableop_1_deep_q_network_1_dense_3_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2╖
AssignVariableOp_2AssignVariableOp2assignvariableop_2_deep_q_network_1_dense_4_kernelIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3╡
AssignVariableOp_3AssignVariableOp0assignvariableop_3_deep_q_network_1_dense_4_biasIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4╖
AssignVariableOp_4AssignVariableOp2assignvariableop_4_deep_q_network_1_dense_5_kernelIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5╡
AssignVariableOp_5AssignVariableOp0assignvariableop_5_deep_q_network_1_dense_5_biasIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_59
NoOpNoOp"/device:CPU:0*
_output_shapes
 2
NoOpф

Identity_6Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^NoOp"/device:CPU:0*
T0*
_output_shapes
: 2

Identity_6c

Identity_7IdentityIdentity_6:output:0^NoOp_1*
T0*
_output_shapes
: 2

Identity_7╬
NoOp_1NoOp^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5*"
_acd_function_control_output(*
_output_shapes
 2
NoOp_1"!

identity_7Identity_7:output:0*!
_input_shapes
: : : : : : : 2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12(
AssignVariableOp_2AssignVariableOp_22(
AssignVariableOp_3AssignVariableOp_32(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_5:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
Р(
л
"__inference__wrapped_model_1904141
input_1K
7deep_q_network_1_dense_3_matmul_readvariableop_resource:
▀АG
8deep_q_network_1_dense_3_biasadd_readvariableop_resource:	АK
7deep_q_network_1_dense_4_matmul_readvariableop_resource:
ААG
8deep_q_network_1_dense_4_biasadd_readvariableop_resource:	АJ
7deep_q_network_1_dense_5_matmul_readvariableop_resource:	АF
8deep_q_network_1_dense_5_biasadd_readvariableop_resource:
identityИв/deep_q_network_1/dense_3/BiasAdd/ReadVariableOpв.deep_q_network_1/dense_3/MatMul/ReadVariableOpв/deep_q_network_1/dense_4/BiasAdd/ReadVariableOpв.deep_q_network_1/dense_4/MatMul/ReadVariableOpв/deep_q_network_1/dense_5/BiasAdd/ReadVariableOpв.deep_q_network_1/dense_5/MatMul/ReadVariableOp┌
.deep_q_network_1/dense_3/MatMul/ReadVariableOpReadVariableOp7deep_q_network_1_dense_3_matmul_readvariableop_resource* 
_output_shapes
:
▀А*
dtype020
.deep_q_network_1/dense_3/MatMul/ReadVariableOp└
deep_q_network_1/dense_3/MatMulMatMulinput_16deep_q_network_1/dense_3/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2!
deep_q_network_1/dense_3/MatMul╪
/deep_q_network_1/dense_3/BiasAdd/ReadVariableOpReadVariableOp8deep_q_network_1_dense_3_biasadd_readvariableop_resource*
_output_shapes	
:А*
dtype021
/deep_q_network_1/dense_3/BiasAdd/ReadVariableOpц
 deep_q_network_1/dense_3/BiasAddBiasAdd)deep_q_network_1/dense_3/MatMul:product:07deep_q_network_1/dense_3/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2"
 deep_q_network_1/dense_3/BiasAddд
deep_q_network_1/dense_3/ReluRelu)deep_q_network_1/dense_3/BiasAdd:output:0*
T0*(
_output_shapes
:         А2
deep_q_network_1/dense_3/Relu┌
.deep_q_network_1/dense_4/MatMul/ReadVariableOpReadVariableOp7deep_q_network_1_dense_4_matmul_readvariableop_resource* 
_output_shapes
:
АА*
dtype020
.deep_q_network_1/dense_4/MatMul/ReadVariableOpф
deep_q_network_1/dense_4/MatMulMatMul+deep_q_network_1/dense_3/Relu:activations:06deep_q_network_1/dense_4/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2!
deep_q_network_1/dense_4/MatMul╪
/deep_q_network_1/dense_4/BiasAdd/ReadVariableOpReadVariableOp8deep_q_network_1_dense_4_biasadd_readvariableop_resource*
_output_shapes	
:А*
dtype021
/deep_q_network_1/dense_4/BiasAdd/ReadVariableOpц
 deep_q_network_1/dense_4/BiasAddBiasAdd)deep_q_network_1/dense_4/MatMul:product:07deep_q_network_1/dense_4/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2"
 deep_q_network_1/dense_4/BiasAddд
deep_q_network_1/dense_4/ReluRelu)deep_q_network_1/dense_4/BiasAdd:output:0*
T0*(
_output_shapes
:         А2
deep_q_network_1/dense_4/Relu┘
.deep_q_network_1/dense_5/MatMul/ReadVariableOpReadVariableOp7deep_q_network_1_dense_5_matmul_readvariableop_resource*
_output_shapes
:	А*
dtype020
.deep_q_network_1/dense_5/MatMul/ReadVariableOpу
deep_q_network_1/dense_5/MatMulMatMul+deep_q_network_1/dense_4/Relu:activations:06deep_q_network_1/dense_5/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2!
deep_q_network_1/dense_5/MatMul╫
/deep_q_network_1/dense_5/BiasAdd/ReadVariableOpReadVariableOp8deep_q_network_1_dense_5_biasadd_readvariableop_resource*
_output_shapes
:*
dtype021
/deep_q_network_1/dense_5/BiasAdd/ReadVariableOpх
 deep_q_network_1/dense_5/BiasAddBiasAdd)deep_q_network_1/dense_5/MatMul:product:07deep_q_network_1/dense_5/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2"
 deep_q_network_1/dense_5/BiasAddД
IdentityIdentity)deep_q_network_1/dense_5/BiasAdd:output:0^NoOp*
T0*'
_output_shapes
:         2

Identityў
NoOpNoOp0^deep_q_network_1/dense_3/BiasAdd/ReadVariableOp/^deep_q_network_1/dense_3/MatMul/ReadVariableOp0^deep_q_network_1/dense_4/BiasAdd/ReadVariableOp/^deep_q_network_1/dense_4/MatMul/ReadVariableOp0^deep_q_network_1/dense_5/BiasAdd/ReadVariableOp/^deep_q_network_1/dense_5/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 2b
/deep_q_network_1/dense_3/BiasAdd/ReadVariableOp/deep_q_network_1/dense_3/BiasAdd/ReadVariableOp2`
.deep_q_network_1/dense_3/MatMul/ReadVariableOp.deep_q_network_1/dense_3/MatMul/ReadVariableOp2b
/deep_q_network_1/dense_4/BiasAdd/ReadVariableOp/deep_q_network_1/dense_4/BiasAdd/ReadVariableOp2`
.deep_q_network_1/dense_4/MatMul/ReadVariableOp.deep_q_network_1/dense_4/MatMul/ReadVariableOp2b
/deep_q_network_1/dense_5/BiasAdd/ReadVariableOp/deep_q_network_1/dense_5/BiasAdd/ReadVariableOp2`
.deep_q_network_1/dense_5/MatMul/ReadVariableOp.deep_q_network_1/dense_5/MatMul/ReadVariableOp:Q M
(
_output_shapes
:         ▀
!
_user_specified_name	input_1
Д
И
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904323	
state:
&dense_3_matmul_readvariableop_resource:
▀А6
'dense_3_biasadd_readvariableop_resource:	А:
&dense_4_matmul_readvariableop_resource:
АА6
'dense_4_biasadd_readvariableop_resource:	А9
&dense_5_matmul_readvariableop_resource:	А5
'dense_5_biasadd_readvariableop_resource:
identityИвdense_3/BiasAdd/ReadVariableOpвdense_3/MatMul/ReadVariableOpвdense_4/BiasAdd/ReadVariableOpвdense_4/MatMul/ReadVariableOpвdense_5/BiasAdd/ReadVariableOpвdense_5/MatMul/ReadVariableOpз
dense_3/MatMul/ReadVariableOpReadVariableOp&dense_3_matmul_readvariableop_resource* 
_output_shapes
:
▀А*
dtype02
dense_3/MatMul/ReadVariableOpЛ
dense_3/MatMulMatMulstate%dense_3/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_3/MatMulе
dense_3/BiasAdd/ReadVariableOpReadVariableOp'dense_3_biasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02 
dense_3/BiasAdd/ReadVariableOpв
dense_3/BiasAddBiasAdddense_3/MatMul:product:0&dense_3/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_3/BiasAddq
dense_3/ReluReludense_3/BiasAdd:output:0*
T0*(
_output_shapes
:         А2
dense_3/Reluз
dense_4/MatMul/ReadVariableOpReadVariableOp&dense_4_matmul_readvariableop_resource* 
_output_shapes
:
АА*
dtype02
dense_4/MatMul/ReadVariableOpа
dense_4/MatMulMatMuldense_3/Relu:activations:0%dense_4/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_4/MatMulе
dense_4/BiasAdd/ReadVariableOpReadVariableOp'dense_4_biasadd_readvariableop_resource*
_output_shapes	
:А*
dtype02 
dense_4/BiasAdd/ReadVariableOpв
dense_4/BiasAddBiasAdddense_4/MatMul:product:0&dense_4/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:         А2
dense_4/BiasAddq
dense_4/ReluReludense_4/BiasAdd:output:0*
T0*(
_output_shapes
:         А2
dense_4/Reluж
dense_5/MatMul/ReadVariableOpReadVariableOp&dense_5_matmul_readvariableop_resource*
_output_shapes
:	А*
dtype02
dense_5/MatMul/ReadVariableOpЯ
dense_5/MatMulMatMuldense_4/Relu:activations:0%dense_5/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2
dense_5/MatMulд
dense_5/BiasAdd/ReadVariableOpReadVariableOp'dense_5_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02 
dense_5/BiasAdd/ReadVariableOpб
dense_5/BiasAddBiasAdddense_5/MatMul:product:0&dense_5/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:         2
dense_5/BiasAdds
IdentityIdentitydense_5/BiasAdd:output:0^NoOp*
T0*'
_output_shapes
:         2

IdentityС
NoOpNoOp^dense_3/BiasAdd/ReadVariableOp^dense_3/MatMul/ReadVariableOp^dense_4/BiasAdd/ReadVariableOp^dense_4/MatMul/ReadVariableOp^dense_5/BiasAdd/ReadVariableOp^dense_5/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*3
_input_shapes"
 :         ▀: : : : : : 2@
dense_3/BiasAdd/ReadVariableOpdense_3/BiasAdd/ReadVariableOp2>
dense_3/MatMul/ReadVariableOpdense_3/MatMul/ReadVariableOp2@
dense_4/BiasAdd/ReadVariableOpdense_4/BiasAdd/ReadVariableOp2>
dense_4/MatMul/ReadVariableOpdense_4/MatMul/ReadVariableOp2@
dense_5/BiasAdd/ReadVariableOpdense_5/BiasAdd/ReadVariableOp2>
dense_5/MatMul/ReadVariableOpdense_5/MatMul/ReadVariableOp:O K
(
_output_shapes
:         ▀

_user_specified_namestate
°
Щ
)__inference_dense_4_layer_call_fn_1904421

inputs
unknown:
АА
	unknown_0:	А
identityИвStatefulPartitionedCallї
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:         А*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8В *M
fHRF
D__inference_dense_4_layer_call_and_return_conditional_losses_19041762
StatefulPartitionedCall|
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:         А2

Identityh
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 2
NoOp"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:         А: : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:         А
 
_user_specified_nameinputs"иL
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*м
serving_defaultШ
<
input_11
serving_default_input_1:0         ▀<
output_10
StatefulPartitionedCall:0         tensorflow/serving/predict:┤>
Ж
fc1
fc2
fc3
	optimizer
loss
	variables
regularization_losses
trainable_variables
		keras_api


signatures
*1&call_and_return_all_conditional_losses
2_default_save_signature
3__call__"
_tf_keras_model
╗

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
*4&call_and_return_all_conditional_losses
5__call__"
_tf_keras_layer
╗

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
*6&call_and_return_all_conditional_losses
7__call__"
_tf_keras_layer
╗

kernel
bias
	variables
trainable_variables
regularization_losses
	keras_api
*8&call_and_return_all_conditional_losses
9__call__"
_tf_keras_layer
"
	optimizer
 "
trackable_dict_wrapper
J
0
1
2
3
4
5"
trackable_list_wrapper
 "
trackable_list_wrapper
J
0
1
2
3
4
5"
trackable_list_wrapper
╩
	variables
metrics
non_trainable_variables
layer_regularization_losses
regularization_losses
 layer_metrics

!layers
trainable_variables
3__call__
2_default_save_signature
*1&call_and_return_all_conditional_losses
&1"call_and_return_conditional_losses"
_generic_user_object
,
:serving_default"
signature_map
3:1
▀А2deep_q_network_1/dense_3/kernel
,:*А2deep_q_network_1/dense_3/bias
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
н
	variables
"metrics
#non_trainable_variables
$layer_regularization_losses
trainable_variables
%layer_metrics

&layers
regularization_losses
5__call__
*4&call_and_return_all_conditional_losses
&4"call_and_return_conditional_losses"
_generic_user_object
3:1
АА2deep_q_network_1/dense_4/kernel
,:*А2deep_q_network_1/dense_4/bias
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
н
	variables
'metrics
(non_trainable_variables
)layer_regularization_losses
trainable_variables
*layer_metrics

+layers
regularization_losses
7__call__
*6&call_and_return_all_conditional_losses
&6"call_and_return_conditional_losses"
_generic_user_object
2:0	А2deep_q_network_1/dense_5/kernel
+:)2deep_q_network_1/dense_5/bias
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
н
	variables
,metrics
-non_trainable_variables
.layer_regularization_losses
trainable_variables
/layer_metrics

0layers
regularization_losses
9__call__
*8&call_and_return_all_conditional_losses
&8"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
5
0
1
2"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
┼2┬
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904323
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904347б
Ш▓Ф
FullArgSpec
argsЪ
jself
jstate
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
═B╩
"__inference__wrapped_model_1904141input_1"Ш
С▓Н
FullArgSpec
argsЪ 
varargsjargs
varkwjkwargs
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
П2М
2__inference_deep_q_network_1_layer_call_fn_1904364
2__inference_deep_q_network_1_layer_call_fn_1904381б
Ш▓Ф
FullArgSpec
argsЪ
jself
jstate
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
ю2ы
D__inference_dense_3_layer_call_and_return_conditional_losses_1904392в
Щ▓Х
FullArgSpec
argsЪ
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
╙2╨
)__inference_dense_3_layer_call_fn_1904401в
Щ▓Х
FullArgSpec
argsЪ
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
ю2ы
D__inference_dense_4_layer_call_and_return_conditional_losses_1904412в
Щ▓Х
FullArgSpec
argsЪ
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
╙2╨
)__inference_dense_4_layer_call_fn_1904421в
Щ▓Х
FullArgSpec
argsЪ
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
ю2ы
D__inference_dense_5_layer_call_and_return_conditional_losses_1904431в
Щ▓Х
FullArgSpec
argsЪ
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
╙2╨
)__inference_dense_5_layer_call_fn_1904440в
Щ▓Х
FullArgSpec
argsЪ
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 
╠B╔
%__inference_signature_wrapper_1904299input_1"Ф
Н▓Й
FullArgSpec
argsЪ 
varargs
 
varkwjkwargs
defaults
 

kwonlyargsЪ 
kwonlydefaults
 
annotationsк *
 Ц
"__inference__wrapped_model_1904141p1в.
'в$
"К
input_1         ▀
к "3к0
.
output_1"К
output_1         ▒
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904323`/в,
%в"
 К
state         ▀
к "%в"
К
0         
Ъ │
M__inference_deep_q_network_1_layer_call_and_return_conditional_losses_1904347b1в.
'в$
"К
input_1         ▀
к "%в"
К
0         
Ъ Л
2__inference_deep_q_network_1_layer_call_fn_1904364U1в.
'в$
"К
input_1         ▀
к "К         Й
2__inference_deep_q_network_1_layer_call_fn_1904381S/в,
%в"
 К
state         ▀
к "К         ж
D__inference_dense_3_layer_call_and_return_conditional_losses_1904392^0в-
&в#
!К
inputs         ▀
к "&в#
К
0         А
Ъ ~
)__inference_dense_3_layer_call_fn_1904401Q0в-
&в#
!К
inputs         ▀
к "К         Аж
D__inference_dense_4_layer_call_and_return_conditional_losses_1904412^0в-
&в#
!К
inputs         А
к "&в#
К
0         А
Ъ ~
)__inference_dense_4_layer_call_fn_1904421Q0в-
&в#
!К
inputs         А
к "К         Ае
D__inference_dense_5_layer_call_and_return_conditional_losses_1904431]0в-
&в#
!К
inputs         А
к "%в"
К
0         
Ъ }
)__inference_dense_5_layer_call_fn_1904440P0в-
&в#
!К
inputs         А
к "К         д
%__inference_signature_wrapper_1904299{<в9
в 
2к/
-
input_1"К
input_1         ▀"3к0
.
output_1"К
output_1         
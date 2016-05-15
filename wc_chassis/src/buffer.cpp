
#include "iostream"
#include "string.h"

#include "buffer.h"

void Init(BufList* list,int iLen){

    if (list->m_pBuffer != NULL)
	{

        delete list->m_pBuffer;
        list->m_pBuffer = NULL;
	}

	if (iLen>0 && iLen<1025)
	{
	
		list->m_pBuffer = (unsigned char*)malloc(iLen*sizeof(unsigned char));
		memset(list->m_pBuffer,0,iLen);
		list->m_iMax = iLen;
	}
}
void Des(BufList* list){
    if (list->m_pBuffer != NULL)
	{
		free(list->m_pBuffer);
        list->m_pBuffer = NULL;
	}
}
int Size(BufList* list){

	return list->m_iOffset;
}
int Write(BufList* list,unsigned char* pWrite,int len){

	if ( len > (list->m_iMax - list->m_iOffset) )
	{
		len = list->m_iMax - list->m_iOffset;
	}

    if (list->m_pBuffer == NULL)
	{
		//LOGS_ERROR("TransferDevice")<<"write bufferP is null :"<<len;
		return 0;
	}
    assert(list->m_pBuffer!=NUll);
	memset(list->m_pBuffer + list->m_iOffset,0,len);
	memcpy(list->m_pBuffer + list->m_iOffset,pWrite,len);

	list->m_iOffset+=len;

	return len;
};
int Read(BufList* list,unsigned char* pRead,int* len){

	if (list->m_iOffset>0)
	{
		assert(list->m_pBuffer!=NULL);
		memset(pRead,0,list->m_iOffset);
		memcpy(pRead,list->m_pBuffer,list->m_iOffset);
		*len = list->m_iOffset;

		if (list->m_pBuffer != NULL)
		{
			memset(list->m_pBuffer,0,list->m_iMax);
		}

		list->m_iOffset = 0;

		return *len;
	}
	else
	{
		return 0;
	}
}
void Clear(BufList* list){

	if (list->m_pBuffer != NULL)
	{
		memset(list->m_pBuffer,0,list->m_iMax);
	}

	list->m_iOffset = 0;
}
int IsPull(BufList* list){

	if (list->m_iOffset == list->m_iMax )
	{
		return 1;
	}
	else
	{
		return 0;
	}

}
void Print(BufList* list){
  int i = 0;
  std::cout<<"buf size:"<<(int)list->m_iOffset;
  for(;i < list->m_iOffset; ++i){

    std::cout<<"buf["<<i<<"]:"<<(int)list->m_pBuffer[i];
  }
  std::cout<<std::endl;
}

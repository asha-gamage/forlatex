from __future__ import print_function
import torch
from model import highwayNet
from utils_260323 import ngsimDataset,maskedNLL,maskedMSETest,maskedNLLTest
from torch.utils.data import DataLoader
import time
import matplotlib.pyplot as plt # Addition
import numpy as np #Addition
import xlsxwriter # import xlsxwriter module

def main():
    ## Network Arguments
    args = {}
    args['use_cuda'] = False
    args['encoder_size'] = 64
    args['decoder_size'] = 128
    args['in_length'] = 16
    args['out_length'] = 25
    args['grid_size'] = (13,3)
    args['soc_conv_depth'] = 64
    args['conv_3x1_depth'] = 16
    args['dyn_embedding_size'] = 32
    args['input_embedding_size'] = 32
    args['num_lat_classes'] = 3
    args['num_lon_classes'] = 2
    args['use_maneuvers'] = False
    args['train_flag'] = False

    # Evaluation metric:
    metric = 'rmse'  #or nll

    # Initialize network
    net = highwayNet(args)
    net.load_state_dict(torch.load('trained_models/cslstm_no_m.tar'))
    # net.load_state_dict(torch.load('trained_models/cslstm_m.tar'))
    # net.load_state_dict(torch.load('trained_models/simData/cslstm_no_m_230523.tar', map_location=torch.device('cpu')))
    if args['use_cuda']:
        net = net.cuda()

    path = 'US101_original/radarData/manoeuvre_RLC/Range/relVel_10kph/'
    # path = 'Road Curve/radarData/manoeuvre_RLC/Range/relVel_5kph/'
    f = open(path+'RMSE_US101_RLC.txt', 'w') # Open a text file to save the calculated RMSE values at each range setting
    # f = open(path + 'RMSE_Curve_RLC.txt', 'w')  # Open a text file to save the calculated RMSE values at each range setting
    book = xlsxwriter.Workbook(path+'RMSE_US101_RLC.xlsx')
    # book = xlsxwriter.Workbook(path + 'RMSE_Curve_RLC.xlsx')
    sheet = book.add_worksheet()
    cell_format = book.add_format({'bold': True, 'text_wrap': True, 'valign': 'top'})

    for a in range(25, 151, 25):
        tsSet = ngsimDataset(path + 'matFiles/US101_original_Range_' + str(a) + ' occluded.mat')
        # tsSet = ngsimDataset(path + 'matFiles/Road Curve_Range_' + str(a) + '.mat')
        tsDataloader = DataLoader(tsSet, batch_size=1, shuffle=True, num_workers=1, collate_fn=tsSet.collate_fn,drop_last=True)

        lossVals = torch.zeros(25)#.cuda()
        counts = torch.zeros(25)#.cuda()

        for i, data in enumerate(tsDataloader):
            st_time = time.time()
            hist, nbrs, mask, lat_enc, lon_enc, fut, op_mask = data

            # Initialize Variables
            if args['use_cuda']:
                hist = hist.cuda()
                nbrs = nbrs.cuda()
                mask = mask.cuda()
                lat_enc = lat_enc.cuda()
                lon_enc = lon_enc.cuda()
                fut = fut.cuda()
                op_mask = op_mask.cuda()

            if metric == 'nll':
                # Forward pass
                if args['use_maneuvers']:
                    fut_pred, lat_pred, lon_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                    l,c = maskedNLLTest(fut_pred, lat_pred, lon_pred, fut, op_mask)
                else:
                    fut_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                    l, c = maskedNLLTest(fut_pred, 0, 0, fut, op_mask,use_maneuvers=False)
            else:
                # Forward pass
                if args['use_maneuvers']:
                    fut_pred, lat_pred, lon_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                    fut_pred_max = torch.zeros_like(fut_pred[0])
                    for k in range(lat_pred.shape[0]):
                        lat_man = torch.argmax(lat_pred[k, :]).detach()
                        lon_man = torch.argmax(lon_pred[k, :]).detach()
                        indx = lon_man*3 + lat_man
                        fut_pred_max[:,k,:] = fut_pred[indx][:,k,:]
                    l, c = maskedMSETest(fut_pred_max, fut, op_mask)
                else:
                    fut_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                    l, c = maskedMSETest(fut_pred, fut, op_mask)

            lossVals +=l.detach()
            counts += c.detach()

        # rmse = (torch.pow(lossVals / counts, 0.5) * 0.3048)
        if metric == 'nll':
            nll = (lossVals / counts)
            loss = nll
        else:
            rmse = (torch.pow(lossVals / counts, 0.5) * 0.3048)  # Calculate RMSE and convert from feet to meters
            loss = rmse

        #print('something', file=f)
        print('RMSE at Range ' + str(a), loss)
        print('RMSE at Range ' + str(a), loss, file=f)  # Calculate RMSE and convert from feet to meters

        # Rows and columns are zero indexed.
        row = 1
        column = int(a/25)
        sheet.write(0, column, 'RMSE at Range '+str(a)+ 'm', cell_format)
        # iterating through the content list
        for item in rmse:
            if torch.isnan(item):
                item = 'nan'
            else:
                item = round(item.numpy().item(),2)
            # write operation perform
            sheet.write(row, column, item)

            # incrementing the value of row by one with each iterations.
            row += 1

        # torch.save(rmse, path+'Curve_Range_'+str(a)+'.pt')
        torch.save(rmse, path + 'US101_Range_' + str(a) + ' occluded.pt')
    # book.close()


    # gtSet = ngsimDataset(path + 'matFiles/Road Curve_groundTruth.mat')
    gtSet = ngsimDataset(path + 'matFiles/US101_original_groundTruth occluded.mat')
    # print("Length of Dataset: ", len(tsSet))
    gtDataloader = DataLoader(gtSet, batch_size=1, shuffle=True, num_workers=1, collate_fn=gtSet.collate_fn, drop_last=True)  # Added parameter, 'drop_last' and changed 'num_workers' to 1

    lossVals = torch.zeros(25)  # .cuda() # should .cuda() be removed?
    counts = torch.zeros(25)  # .cuda()

    for i, data in enumerate(gtDataloader):
        st_time = time.time()
        hist, nbrs, mask, lat_enc, lon_enc, fut, op_mask = data

        # Initialize Variables
        if args['use_cuda']:
            hist = hist.cuda()
            nbrs = nbrs.cuda()
            mask = mask.cuda()
            lat_enc = lat_enc.cuda()
            lon_enc = lon_enc.cuda()
            fut = fut.cuda()
            op_mask = op_mask.cuda()

        if metric == 'nll':
            # Forward pass
            if args['use_maneuvers']:
                fut_pred, lat_pred, lon_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                l, c = maskedNLLTest(fut_pred, lat_pred, lon_pred, fut, op_mask)
            else:
                fut_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                l, c = maskedNLLTest(fut_pred, 0, 0, fut, op_mask, use_maneuvers=False)
        else:
            # Forward pass
            if args['use_maneuvers']:
                fut_pred, lat_pred, lon_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                fut_pred_max = torch.zeros_like(fut_pred[0])
                for k in range(lat_pred.shape[0]):
                    lat_man = torch.argmax(lat_pred[k, :]).detach()
                    lon_man = torch.argmax(lon_pred[k, :]).detach()
                    indx = lon_man * 3 + lat_man
                    fut_pred_max[:, k, :] = fut_pred[indx][:, k, :]
                l, c = maskedMSETest(fut_pred_max, fut, op_mask)
            else:
                fut_pred = net(hist, nbrs, mask, lat_enc, lon_enc)
                l, c = maskedMSETest(fut_pred, fut, op_mask)

        lossVals += l.detach()
        counts += c.detach()

    err = (torch.pow(lossVals / counts, 0.5) * 0.3048)
    # if metric == 'nll':
    #     print(lossVals / counts)
    # else:
    #     err = (torch.pow(lossVals / counts, 0.5) * 0.3048)

    print('RMSE on gndTruth: ', err)
    print('RMSE on gndTruth ', err, file=f)  # Calculate RMSE and convert from feet to meters

    row = 1
    sheet.write(0, 0, 'GroundTruth RMSE',cell_format)
    for item in err:
        if torch.isnan(item):
            item = 'nan'
        else:
            item = round(item.numpy().item(), 2)
    # for item in err.numpy():
        # write operation perform
        sheet.write(row, 0, item)
        # incrementing the value of row by one with each iterations.
        row += 1
    # torch.save(err, path + 'Curve_groundTruth.pt')
    torch.save(err, path + 'US101_groundTruth occluded.pt')

    f.close() # Close the text file
    book.close()

if __name__ == "__main__":
    main()
